FeatureScript 2960;

export import(path : "onshape/std/tool.fs", version : "2960.0");
export import(path : "onshape/std/bridgingCurve.fs", version : "2960.0");
import(path : "onshape/std/boolean.fs", version : "2960.0");
import(path : "onshape/std/booleanHeuristics.fs", version : "2960.0");
import(path : "onshape/std/containers.fs", version : "2960.0");
import(path : "onshape/std/coordSystem.fs", version : "2960.0");
import(path : "onshape/std/curveGeometry.fs", version : "2960.0");
import(path : "onshape/std/evaluate.fs", version : "2960.0");
import(path : "onshape/std/feature.fs", version : "2960.0");
import(path : "onshape/std/math.fs", version : "2960.0");
import(path : "onshape/std/path.fs", version : "2960.0");
import(path : "onshape/std/splineUtils.fs", version : "2960.0");
import(path : "onshape/std/surfaceGeometry.fs", version : "2960.0");
import(path : "onshape/std/valueBounds.fs", version : "2960.0");
import(path : "onshape/std/vector.fs", version : "2960.0");
import(path : "onshape/std/debug.fs", version : "2960.0");


/**
 * Sweeps a thread profile along a bridging curve that transitions from a helix endpoint
 * to a point inside the cylinder, then trims at the cylinder surface — creating a smooth
 * thread run-out. Non-cyliner surfaces are also supported.
 */
annotation { "Feature Type Name" : "Thread Terminus" }
export const threadTerminus = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        booleanStepTypePredicate(definition);

        annotation { "Name" : "Faces and sketch regions to sweep",
                    "Filter" : EntityType.FACE && GeometryType.PLANE && ConstructionObject.NO }
        definition.profile is Query;

        annotation { "Name" : "Path to extend",
                    "Filter" : (EntityType.EDGE && ConstructionObject.NO) || (EntityType.BODY && BodyType.WIRE && SketchObject.NO),
                    "MaxNumberOfPicks" : 1 }
        definition.path is Query;

        annotation { "Name" : "Vertex on path",
                    "Filter" : EntityType.VERTEX,
                    "MaxNumberOfPicks" : 1 }
        definition.pathVertex is Query;

        annotation { "Name" : "Surface to taper into",
                    "Filter" : (EntityType.BODY && BodyType.SHEET && SketchObject.NO) || EntityType.FACE || BodyType.MATE_CONNECTOR,
                    "MaxNumberOfPicks" : 1 }
        definition.surface is Query;

        annotation { "Name" : "Taper length" }
        isLength(definition.taperLength, NONNEGATIVE_LENGTH_BOUNDS);

        annotation { "Name" : "Match", "UIHint" : UIHint.REMEMBER_PREVIOUS_VALUE }
        definition.match1 is BridgingCurveMatchType;

        annotation { "Name" : "Edit control points", "UIHint" : UIHint.REMEMBER_PREVIOUS_VALUE }
        definition.editControlPoints is boolean;

        if (definition.editControlPoints)
        {
            annotation { "Group Name" : "Edit control points", "Collapsed By Default" : false, "Driving Parameter" : "editControlPoints" }
            {
                annotation { "Name" : "Start magnitude" }
                isReal(definition.side1Magnitude, POSITIVE_REAL_BOUNDS);

                if (definition.match1 == BridgingCurveMatchType.CURVATURE || definition.match1 == BridgingCurveMatchType.G3)
                {
                    annotation { "Name" : "Start curvature offset" }
                    isReal(definition.side1CurvatureOffset, CLAMP_MAGNITUDE_REAL_BOUNDS);
                }
                if (definition.match1 == BridgingCurveMatchType.G3)
                {
                    annotation { "Name" : "Start flow offset" }
                    isReal(definition.side1G3Offset, CLAMP_MAGNITUDE_REAL_BOUNDS);
                }
            }
        }

        booleanStepScopePredicate(definition);
    }
    {
        verifyNonemptyQuery(context, definition, "profile", ErrorStringEnum.SWEEP_SELECT_PROFILE);
        verifyNonemptyQuery(context, definition, "path", ErrorStringEnum.SWEEP_SELECT_PATH);
        verifyNonemptyQuery(context, definition, "pathVertex", "Select vertex on path");
        verifyNonemptyQuery(context, definition, "surface", "Select surface to taper into");

        // Step 1: find the path edge at the selected vertex
        //
        // Edges adjacent to the vertex via vertex adjacency = edges that have the vertex as endpoint
        const bodyEdges = qEntityFilter(definition.path, EntityType.BODY)->qOwnedByBody(EntityType.EDGE);
        const directEdge = qEntityFilter(definition.path, EntityType.EDGE);
        const allEdges = qUnion([bodyEdges, directEdge]);
        const p0_edge = qIntersection([
                    qAdjacent(definition.pathVertex, AdjacencyType.VERTEX, EntityType.EDGE),
                    allEdges
                ]);

        // Determine whether the vertex is at parameter 0 or 1 on the edge
        const p0 = evVertexPoint(context, { "vertex" : definition.pathVertex });
        const p0_parameter = evDistance(context, { "side0" : p0, "side1" : p0_edge }).sides[1].parameter;

        // Step 2: extract G3 curvature data at the selected vertex
        //
        const curvatureResult = evEdgeCurvature(context, {
                    "edge" : p0_edge,
                    "parameter" : p0_parameter,
                    "arcLengthParameterization" : false
                });
        const kPrimeRaw = evEdgeCurvatureDerivative(context, {
                    "edge" : p0_edge,
                    "parameter" : p0_parameter
                });

        // If the vertex is at the edge start (param 0) the tangent points points towards the path, so flip the tangent
        // to continue past the endpoint in the correct direction.
        const flip = (p0_parameter == 0);
        const p0_tangent = flip ? -curvatureFrameTangent(curvatureResult) : curvatureFrameTangent(curvatureResult);
        const p0_kPrime = flip ? -kPrimeRaw : kPrimeRaw;
        const p0_curveNormal = curvatureFrameNormal(curvatureResult);

        // Step 3: surface normal at p0
        //
        const surfaceData = resolveSurface(context, id, definition.surface, p0, definition.profile);

        // Step 4: profile depth and p1
        //
        // Bounding box in profile-plane frame: xAxis = surfaceData.normal projected into profile plane,
        // zAxis = profile plane normal. maxCorner[0] gives the outward depth of the thread tip.
        const profileNormal = evPlane(context, { "face" : definition.profile }).normal;
        const xAxis = normalize(surfaceData.normal - dot(surfaceData.normal, profileNormal) * profileNormal);
        const profileBox = evBox3d(context, {
                    "topology" : definition.profile,
                    "cSys" : coordSystem(p0, xAxis, profileNormal),
                    "tight" : true
                });
        const p1 = p0 + xAxis * profileBox.maxCorner[0];
        // debug(context, profileBox, coordSystem(p0, xAxis, profileNormal));

        // Steps 5–6: find p2 on the surface at arc distance taperLength from q0
        //
        // Cylinder case: decompose into axial + circumferential components using exact helix geometry (more accurate).
        // General case: intersect a section plane with the surface and walk the resulting curve.
        const qSingleFace = qEntityFilter(definition.surface, EntityType.FACE);
        const surfaceDef = evaluateQueryCount(context, qSingleFace) == 1 ?
            evSurfaceDefinition(context, { "face" : qSingleFace }) : undefined;
        var p2;
        if (surfaceDef is Cylinder)
        {
            p2 = computeP2ForHelix(context, p0, p0_tangent, surfaceDef, definition.taperLength);
        }
        else
        {
            // Step 5: intersect a plane (spanned by surfaceData.normal and p0_tangent) with the surface
            //
            opPlane(context, id + "intersectionPlane", {
                        "plane" : plane(surfaceData.q0, normalize(cross(p0_tangent, surfaceData.normal))),
                    });
            opIntersectFaces(context, id + "surfaceCurve", {
                        "tools" : qCreatedBy(id + "intersectionPlane", EntityType.FACE),
                        "targets" : surfaceData.surfaceFaces
                    });
            const qSurfaceCurveEdges = qCreatedBy(id + "surfaceCurve", EntityType.EDGE);

            // Step 6: walk the surface curve to find p2 at arc distance taperLength from q0
            //
            const surfacePath = constructPath(context, qSurfaceCurveEdges);
            const pathLength = evPathLength(context, surfacePath);

            const q0_parameter = evDistancePath(context, {
                            "side0" : surfaceData.q0,
                            "side1" : surfacePath
                        }).sides[1].parameter;

            const q0_tangent = evPathTangentLines(context, surfacePath, [q0_parameter]).tangentLines[0].direction;
            const sign = dot(q0_tangent, p0_tangent) > 0 ? 1 : -1;
            p2 = evPathTangentLines(context, surfacePath,
                    [q0_parameter + sign * definition.taperLength / pathLength]
                    ).tangentLines[0].origin;

            opDeleteBodies(context, id + "deleteIntersectionPlane", {
                        "entities" : qCreatedBy(id + "intersectionPlane", EntityType.BODY)
                    });
            opDeleteBodies(context, id + "deleteSurfaceCurve", {
                        "entities" : qCreatedBy(id + "surfaceCurve", EntityType.BODY)
                    });
        }

        // Step 7: build bridging curve control points
        //
        const degree = switch (definition.match1) {
                    BridgingCurveMatchType.POSITION : 0,
                    BridgingCurveMatchType.TANGENCY : 1,
                    BridgingCurveMatchType.CURVATURE : 2,
                    BridgingCurveMatchType.G3 : 3 };
        var side1 = {
                "degree" : degree,
                "position" : p1,
                "tangent" : p0_tangent,
                "curvatureDirection" : p0_curveNormal,
                "curvature" : curvatureResult.curvature,
                "normal" : p0_curveNormal,
                "kPrime" : p0_kPrime
            } as BridgingSideData;
        if (definition.editControlPoints)
        {
            side1.speedScale = definition.side1Magnitude;
            side1.curvatureOffsetScale = definition.side1CurvatureOffset;
            side1.g3OffsetScale = definition.side1G3Offset;
        }

        const side2 = {
                    "degree" : 0,
                    "position" : p2
                } as BridgingSideData;

        const controlPoints = computeBridgingControlPoints(context, side1, side2);
        const bCurve = bSplineCurve({
                    "degree" : size(controlPoints) - 1,
                    "isPeriodic" : false,
                    "controlPoints" : controlPoints
                });

        // Step 8: sweep the actual taper body and split excess
        //
        const buildTaperBody = function(opId is Id)
            {
                opCreateBSplineCurve(context, opId + "bridgingCurve", { "bSplineCurve" : bCurve });

                opSweep(context, opId + "sweep", {
                            "profiles" : definition.profile,
                            "path" : qCreatedBy(opId + "bridgingCurve", EntityType.EDGE)
                        });
                // debug(context, qCreatedBy(opId + "bridgingCurve", EntityType.EDGE));

                opSplitPart(context, opId + "split", {
                            "targets" : qBodyType(qCreatedBy(opId + "sweep", EntityType.BODY), BodyType.SOLID),
                            // Doesn't accept multiple faces, so surfaceData.surfaceFaces can't be used when it came
                            // from a sheet with multiple faces. But the original query works fine.
                            "tool" : definition.surface,
                            "keepTools" : true,
                        });

                // opSplitPart does not create new bodies but modifies the existing one, so `qCreatedBy(opId + "split",
                // EntityType.BODY)` would be empty.
                // p1 is outside the surface so keep the body closest to it.
                const qKeep = qCreatedBy(opId + "sweep", EntityType.BODY)->qClosestTo(p1);
                opDeleteBodies(context, opId + "deleteExcess", {
                            "entities" : qCreatedBy(opId + "sweep", EntityType.BODY)->qSubtraction(qKeep)
                        });
                opDeleteBodies(context, opId + "deleteBridgingCurve", {
                            "entities" : qCreatedBy(opId + "bridgingCurve", EntityType.BODY)
                        });

                const tmpSurfacePlane = qCreatedBy(id + "surfacePlane", EntityType.BODY);
                if (!isQueryEmpty(context, tmpSurfacePlane))
                    opDeleteBodies(context, opId + "deleteSurfacePlane", { "entities" : tmpSurfacePlane });
            };

        buildTaperBody(id);

        processNewBodyIfNeeded(context, id, definition, buildTaperBody);
    },
    {
            operationType : NewBodyOperationType.NEW,
            match1 : BridgingCurveMatchType.CURVATURE,
            editControlPoints : false,
            side1Magnitude : 1,
            side1CurvatureOffset : 1,
            side1G3Offset : 1
        }
    );

/**
 * Returns `{ "q0": Vector, "normal": Vector, "surfaceFaces": Query }`.
 * Handles multiple selection types for `qSurface`:
 *   - SOLID face — `evFaceTangentPlane` normal is outward by convention, used as-is
 *   - MATE_CONNECTOR — zAxis as normal (user can flip mate connector in UI); creates `id + "surfacePlane"`
 *   - Construction plane / non-solid sheet — normal oriented toward profile centroid
 */
function resolveSurface(context is Context, id is Id, qSurface is Query, p0 is Vector, qProfile is Query) returns map
{
    if (!isQueryEmpty(context, qBodyType(qSurface, BodyType.MATE_CONNECTOR)))
    {
        const cSys = evMateConnector(context, { "mateConnector" : qSurface });
        const normal = dot(p0 - cSys.origin, cSys.zAxis) >= 0 ? cSys.zAxis : -cSys.zAxis;
        const q0 = p0 - dot(p0 - cSys.origin, normal) * normal;
        opPlane(context, id + "surfacePlane", { "plane" : plane(cSys.origin, normal) });
        return {
                "q0" : q0,
                "normal" : normal,
                "surfaceFaces" : qCreatedBy(id + "surfacePlane", EntityType.FACE)
            };
    }

    const qAllFaces = !isQueryEmpty(context, qEntityFilter(qSurface, EntityType.FACE))
        ? qSurface
        : qOwnedByBody(qSurface, EntityType.FACE);

    const allFaces = evaluateQuery(context, qAllFaces);
    const distResult = evDistance(context, { "side0" : p0, "side1" : qAllFaces });
    const q0 = distResult.sides[1].point;
    const closestFace = allFaces[distResult.sides[1].index];
    var normal = evFaceTangentPlane(context, {
                "face" : closestFace,
                "parameter" : distResult.sides[1].parameter
            }).normal;

    // Construction plane or non-solid sheet: normal has arbitrary orientation, so orient toward profile
    if (isQueryEmpty(context, qBodyType(qOwnerBody(closestFace), BodyType.SOLID)))
    {
        const profileWorldBox = evBox3d(context, { "topology" : qProfile, "tight" : true });
        const profilePoint = (profileWorldBox.minCorner + profileWorldBox.maxCorner) / 2;
        normal = dot(profilePoint - q0, normal) >= 0 ? normal : -normal;
    }

    return {
            "q0" : q0,
            "normal" : normal,
            "surfaceFaces" : qAllFaces,
        };
}

/**
 * Computes p2 (the G0 bridging curve endpoint) for the cylinder/helix case by decomposing
 * `taperLength` into axial and circumferential components using exact cylinder geometry.
 */
function computeP2ForHelix(
    context is Context,
    p0 is Vector,
    p0_tangent is Vector,
    cylinderDef is map,
    taperLength is ValueWithUnits
) returns Vector
{
    const cylinderAxisDir = cylinderDef.coordSystem.zAxis;
    const cylinderAxisOrigin = cylinderDef.coordSystem.origin;
    const cylinderRadius = cylinderDef.radius;

    const p0_axialProjection = cylinderAxisOrigin + dot(p0 - cylinderAxisOrigin, cylinderAxisDir) * cylinderAxisDir;
    const p0_radial = p0 - p0_axialProjection;
    const p0_radialDir = p0_radial / norm(p0_radial);
    const p0_circularDir = cross(cylinderAxisDir, p0_radialDir);

    const axialAdvance = taperLength * dot(p0_tangent, cylinderAxisDir);
    const circularAdvanceAngle = taperLength * dot(p0_tangent, p0_circularDir) / norm(p0_radial) * radian;
    const p2_radialDir = rotationMatrix3d(cylinderAxisDir, circularAdvanceAngle) * p0_radialDir;
    return p0_axialProjection + axialAdvance * cylinderAxisDir + cylinderRadius * p2_radialDir;
}

/**
 * Like `evDistance` but accepts a `Path` for `side0` or `side1`, returning an arc-length
 * parameter (0..1 along the full path) instead of an edge-local parameter.
 * Can be used in conjection with evPathTangentLines.
 * Adapted from: https://forum.onshape.com/discussion/comment/109246/#Comment_109246
 */
function evDistancePath(context is Context, definition is map) returns map
{
    var def = definition;
    const side0IsPath = definition.side0 is Path;
    const side1IsPath = definition.side1 is Path;
    if (side0IsPath)
    {
        def.side0 = qUnion(definition.side0.edges);
    }
    if (side1IsPath)
    {
        def.side1 = qUnion(definition.side1.edges);
    }

    var result = evDistance(context, def);
    if (side0IsPath)
    {
        result.sides[0].parameter = pathParameterFromEdge(
            context,
            definition.side0,
            definition.side0.edges[result.sides[0].index],
            result.sides[0].parameter);
    }
    if (side1IsPath)
    {
        result.sides[1].parameter = pathParameterFromEdge(
            context,
            definition.side1,
            definition.side1.edges[result.sides[1].index],
            result.sides[1].parameter);
    }
    return result;
}

/**
 * Returns the arc-length parameter (0..1) of `edgeParam` (a parameter on `edge`) within `path`.
 * Accounts for `path.flipped` so the result is consistent with `evPathTangentLines` parameterization.
 */
function pathParameterFromEdge(context is Context, path is Path, edge is Query, edgeParam is number) returns number
{
    const totalLength = evPathLength(context, path);
    var currentLength = 0 * meter;
    for (var i = 0; i < size(path.edges); i += 1)
    {
        const edgeLength = evLength(context, { "entities" : path.edges[i] });
        if (!isQueryEmpty(context, qIntersection([path.edges[i], edge])))
        {
            const p = path.flipped[i] ? (1 - edgeParam) : edgeParam;
            return currentLength / totalLength + p * edgeLength / totalLength;
        }
        currentLength = currentLength + edgeLength;
    }
    throw "pathParameterFromEdge: edge not found in path";
}
