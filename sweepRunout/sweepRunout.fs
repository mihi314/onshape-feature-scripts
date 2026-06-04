FeatureScript 2960;

export import(path : "onshape/std/tool.fs", version : "2960.0");
export import(path : "onshape/std/bridgingCurve.fs", version : "2960.0");
import(path : "onshape/std/boolean.fs", version : "2960.0");
import(path : "onshape/std/containers.fs", version : "2960.0");
import(path : "onshape/std/curveGeometry.fs", version : "2960.0");
import(path : "onshape/std/evaluate.fs", version : "2960.0");
import(path : "onshape/std/feature.fs", version : "2960.0");
import(path : "onshape/std/path.fs", version : "2960.0");
import(path : "onshape/std/surfaceGeometry.fs", version : "2960.0");
import(path : "onshape/std/valueBounds.fs", version : "2960.0");
import(path : "onshape/std/mathUtils.fs", version : "2960.0");
import(path : "onshape/std/debug.fs", version : "2960.0");
import(path : "onshape/std/manipulator.fs", version : "2960.0");

import(path : "32e5418754a29436e5e98979", version : "8ed2ff7e1c2288dcae116756");
import(path : "690372703e51729b485491c6", version : "2b8eadb20af1fc912fefaeab");


/**
 * Sweeps a thread profile along a bridging curve that transitions from a helix endpoint
 * to a point inside the cylinder, then trims at the cylinder surface — creating a smooth
 * thread run-out. Non-cyliner surfaces are also supported.
 */
annotation {
        "Feature Type Name" : "Sweep Runout",
        "Manipulator Change Function" : "sweepRunoutManipulator"
    }
export const sweepRunout = defineFeature(function(context is Context, id is Id, definition is map)
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
                isReal(definition.side1SpeedScale, POSITIVE_REAL_BOUNDS);

                if (definition.match1 == BridgingCurveMatchType.CURVATURE || definition.match1 == BridgingCurveMatchType.G3)
                {
                    annotation { "Name" : "Start curvature offset" }
                    isReal(definition.side1CurvatureOffsetScale, CLAMP_MAGNITUDE_REAL_BOUNDS);
                }
                if (definition.match1 == BridgingCurveMatchType.G3)
                {
                    annotation { "Name" : "Start flow offset" }
                    isReal(definition.side1G3OffsetScale, CLAMP_MAGNITUDE_REAL_BOUNDS);
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

        var sideData = computeBridgingSideData(context, id, definition);
        if (definition.editControlPoints)
        {
            sideData.side1.speedScale = definition.side1SpeedScale;
            sideData.side1.curvatureOffsetScale = definition.side1CurvatureOffsetScale;
            sideData.side1.g3OffsetScale = definition.side1G3OffsetScale;
        }

        const controlPoints = computeBridgingControlPoints(context, sideData.side1, sideData.side2);
        const bCurve = bSplineCurve({
                    "degree" : size(controlPoints) - 1,
                    "isPeriodic" : false,
                    "controlPoints" : controlPoints
                });

        if (definition.editControlPoints && definition.match1 != BridgingCurveMatchType.POSITION)
        {
            addSide1Manipulators(context, id, definition, controlPoints);
            showControlPoints(context, id, controlPoints);
        }

        // Step 8: sweep the actual taper body and split excess
        //
        const buildTaperBody = function(opId is Id)
            {
                opCreateBSplineCurve(context, opId + "bridgingCurve", { "bSplineCurve" : bCurve });
                // debug(context, qCreatedBy(opId + "bridgingCurve", EntityType.EDGE));

                opSweep(context, opId + "sweep", {
                            "profiles" : definition.profile,
                            "path" : qCreatedBy(opId + "bridgingCurve", EntityType.EDGE)
                        });

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
                const qKeep = qCreatedBy(opId + "sweep", EntityType.BODY)->qClosestTo(sideData.side1.position);
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
            side1SpeedScale : 1,
            side1CurvatureOffsetScale : 1,
            side1G3OffsetScale : 1
        }
    );

/**
 * Returns `{ "side1": BridgingSideData,  "side2": BridgingSideData }`.
 */
function computeBridgingSideData(context is Context, id is Id, definition is map) returns map
{
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

    // Step 7: build bridging side data
    //
    const degree = switch (definition.match1) {
                BridgingCurveMatchType.POSITION : 0,
                BridgingCurveMatchType.TANGENCY : 1,
                BridgingCurveMatchType.CURVATURE : 2,
                BridgingCurveMatchType.G3 : 3 };

    const T = p0_tangent;
    const N = p0_curveNormal;
    const B = cross(p0_tangent, p0_curveNormal);
    const kappa = curvatureResult.curvature;

    var side1;
    if (kappa > TOLERANCE.zeroLength / meter)
    {
        const tau = dot(p0_kPrime, B) / kappa; // torison
        const omega = tau * T + kappa * B; // Darboux vector

        const frame = coordSystem(p0, N, T);
        const p1InFrame = fromWorld(frame, p1);

        const framePrime = matrixWithUnitsFromColumns([-kappa * T + tau * B, -tau * N, kappa * N]);

        const kappaPrime = dot(p0_kPrime, N);
        // Involves the fourth derivative of the curve which we don't easily have access to and assume to be 0
        const tauPrime = 0 / meter ^ 2;
        const omegaPrime = tauPrime * T + kappaPrime * B;

        const framePrimePrime = matrixWithUnitsFromColumns([
                    -kappaPrime * T - (kappa ^ 2 + tau ^ 2) * N + tauPrime * B,
                    kappa * tau * T - tauPrime * N - tau ^ 2 * B,
                    p0_kPrime
                ]);

        const p1Prime = T + framePrime * p1InFrame;
        const p1PrimePrime = kappa * N + framePrimePrime * p1InFrame;
        const T1 = normalize(p1Prime);
        const B1 = normalize(cross(p1Prime, p1PrimePrime));
        const N1 = cross(B1, T1);
        const kappa1 = norm(cross(p1Prime, p1PrimePrime)) / norm(p1Prime) ^ 3;

        const p1PrimePrime_alternate = kappa * N + cross(omegaPrime, p1 - p0) + cross(omega, cross(omega, p1 - p0));
        const T1_alternate = normalize(T + cross(omega, p1 - p0));

        // debug(context, { "frame" : coordSystem(p0, N, T), "curvature" : kappa } as EdgeCurvatureResult, DebugColor.GREEN);
        // debug(context, { "frame" : coordSystem(p1, N1, T1), "curvature" : kappa1 } as EdgeCurvatureResult, DebugColor.CYAN);

        side1 = {
                    "degree" : degree,
                    "position" : p1,
                    "tangent" : T1,
                    "curvatureDirection" : N1,
                    "curvature" : kappa1,
                    "normal" : N1,
                    "kPrime" : p0_kPrime // TODO: this still needs a correction
                } as BridgingSideData;
    }
    else
    {
        side1 = {
                    "degree" : degree,
                    "position" : p1,
                    "tangent" : T,
                    "curvatureDirection" : N,
                    "curvature" : kappa,
                    "normal" : N,
                    "kPrime" : p0_kPrime
                } as BridgingSideData;
    }

    const side2 = {
                "degree" : 0,
                "position" : p2
            } as BridgingSideData;

    return { "side1" : side1, "side2" : side2 };
}

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

const MANIPULATOR_SPEED_SCALE = "manipulatorSpeedScale";
const MANIPULATOR_CURVATURE_OFFSET = "manipulatorCurvatureOffsetScale";
const MANIPULATOR_G3_OFFSET = "manipulatorG3OffsetScale";

/** @internal */
export function sweepRunoutManipulator(context is Context, definition is map, newManipulators is map) returns map
{
    var sideData = computeBridgingSideData(context, ["sweepRunoutManipulator"] as Id, definition);

    // Use unscaled control points for magnitude/G3 formulas.
    // Pre-apply speedScale only for the curvature case (mirrors bridgingCurveManipulator).
    if (newManipulators[MANIPULATOR_CURVATURE_OFFSET] is map)
    {
        sideData.side1.speedScale = definition.side1SpeedScale;
    }

    const controlPoints = computeBridgingControlPoints(context, sideData.side1, sideData.side2);

    const magnitudeManip = newManipulators[MANIPULATOR_SPEED_SCALE];
    if (magnitudeManip is map)
    {
        definition.side1SpeedScale = abs(magnitudeManip.offset) / norm(controlPoints[1] - controlPoints[0]);
    }

    const curvatureManip = newManipulators[MANIPULATOR_CURVATURE_OFFSET];
    if (curvatureManip is map)
    {
        const direction = normalize(controlPoints[1] - controlPoints[0]);
        const base = side1CurvatureBase(definition.match1, controlPoints);
        definition.side1CurvatureOffsetScale = curvatureManip.offset / dot(direction, controlPoints[2] - base);
    }

    const g3Manip = newManipulators[MANIPULATOR_G3_OFFSET];
    if (g3Manip is map)
    {
        definition.side1G3OffsetScale = g3Manip.offset / (definition.side1SpeedScale * norm(controlPoints[1] - controlPoints[0]));
    }

    return definition;
}

/**
 * Adds linear drag handles for side1's magnitude, curvature, and G3 flow offset.
 * Only called when `definition.editControlPoints` is true.
 * Mirrors `addManipulatorsForSide` in bridgingCurve.fs for side1 only (no flip, no side2).
 */
function addSide1Manipulators(context is Context, id is Id, definition is map, controlPoints is array)
{
    if (definition.match1 == BridgingCurveMatchType.POSITION)
        return;

    const direction = normalize(controlPoints[1] - controlPoints[0]);
    addManipulators(context, id, {
                (MANIPULATOR_SPEED_SCALE) : linearManipulator({
                        "base" : controlPoints[0],
                        "direction" : direction,
                        "offset" : norm(controlPoints[1] - controlPoints[0]),
                        "primaryParameterId" : "side1SpeedScale"
                    })
            });

    if (definition.match1 == BridgingCurveMatchType.TANGENCY)
        return;

    const base = side1CurvatureBase(definition.match1, controlPoints);
    addManipulators(context, id, {
                (MANIPULATOR_CURVATURE_OFFSET) : linearManipulator({
                        "base" : base,
                        "direction" : direction,
                        "offset" : dot(direction, controlPoints[2] - base),
                        "style" : ManipulatorStyleEnum.SECONDARY,
                        "primaryParameterId" : "side1CurvatureOffsetScale"
                    })
            });

    if (definition.match1 == BridgingCurveMatchType.CURVATURE)
        return;

    const offsetVec = controlPoints[1] - controlPoints[0];
    const g3offset = definition.side1G3OffsetScale;
    addManipulators(context, id, {
                (MANIPULATOR_G3_OFFSET) : linearManipulator({
                        "base" : controlPoints[3] - offsetVec * g3offset,
                        "direction" : direction,
                        "offset" : g3offset * norm(offsetVec),
                        "style" : ManipulatorStyleEnum.SECONDARY,
                        "primaryParameterId" : "side1G3OffsetScale"
                    })
            });
}

/**
 * Zero-offset base point for the curvature manipulator on side1. Mirrors `curvatureBase` in bridgingCurve.fs. For
 * CURVATURE match the answer is exact; for G3 it applies the degree-elevation inverse.
 */
function side1CurvatureBase(match1 is BridgingCurveMatchType, controlPoints is array) returns Vector
{
    const direction = normalize(controlPoints[1] - controlPoints[0]);
    if (match1 != BridgingCurveMatchType.G3)
    {
        // CURVATURE: degree = 3, elevation matrix is identity
        return controlPoints[2] - direction * dot(controlPoints[2] - controlPoints[1], direction);
    }
    // G3: degree = 4, reducedDegree = 3 (only side1 is G3)
    // partialDegreeElevationMatrix(3, 4, 3)^-1 rows: [1,0,0], [-1/3,4/3,0], [1/3,-4/3,2]
    const r1 = -1 / 3 * controlPoints[0] + 4 / 3 * controlPoints[1];
    const r2 = 1 / 3 * controlPoints[0] - 4 / 3 * controlPoints[1] + 2 * controlPoints[2];
    const reducedBase = r2 - direction * dot(r2 - r1, direction);
    // re-elevate: M row 2 = [0, 1/2, 1/2], so base = 1/2*r1 + 1/2*reducedBase
    return (r1 + reducedBase) / 2;
}

function showControlPoints(context is Context, id is Id, controlPoints is array)
{
    if (!isTopLevelId(id))
        return;
    const controlId = id + "controlPoints";
    startFeature(context, controlId, {});
    try
    {
        opPoint(context, controlId + 0 + "point", { "point" : controlPoints[0] });
        for (var i = 1; i < size(controlPoints); i += 1)
        {
            opPoint(context, controlId + i + "point", { "point" : controlPoints[i] });
            opFitSpline(context, controlId + i + "line", { "points" : [controlPoints[i - 1], controlPoints[i]] });
        }
        const edges = qCreatedBy(controlId, EntityType.EDGE);
        const vertices = qCreatedBy(controlId, EntityType.VERTEX)->qBodyType(BodyType.POINT);
        addDebugEntities(context, qUnion([vertices, edges]), DebugColor.MAGENTA);
    }
    abortFeature(context, controlId);
}

function matrixWithUnitsFromColumns(columns is array) returns MatrixWithUnits
precondition
{
    size(columns) > 0;
    for (var col in columns)
    {
        col is Vector;
        col[0] is ValueWithUnits;
        col[0].unit == columns[0][0].unit;
    }
}
{
    return transpose(matrix(stripUnits(columns))) * getUnitOfValue(columns[0][0]);
}
