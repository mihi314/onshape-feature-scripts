FeatureScript 2960;

export import(path : "onshape/std/tool.fs", version : "2960.0");
import(path : "onshape/std/boolean.fs", version : "2960.0");
import(path : "onshape/std/booleanHeuristics.fs", version : "2960.0");
import(path : "onshape/std/bridgingCurve.fs", version : "2960.0");
import(path : "onshape/std/containers.fs", version : "2960.0");
import(path : "onshape/std/coordSystem.fs", version : "2960.0");
import(path : "onshape/std/curveGeometry.fs", version : "2960.0");
import(path : "onshape/std/evaluate.fs", version : "2960.0");
import(path : "onshape/std/feature.fs", version : "2960.0");
import(path : "onshape/std/math.fs", version : "2960.0");
import(path : "onshape/std/splineUtils.fs", version : "2960.0");
import(path : "onshape/std/surfaceGeometry.fs", version : "2960.0");
import(path : "onshape/std/valueBounds.fs", version : "2960.0");
import(path : "onshape/std/vector.fs", version : "2960.0");
import(path : "onshape/std/debug.fs", version : "2960.0");

/**
 * Sweeps a thread profile along a bridging curve that transitions from a helix endpoint
 * to a point inside the cylinder, then trims at the cylinder surface — creating a smooth
 * thread run-out for 3D printing or injection molding.
 */
annotation { "Feature Type Name" : "Thread Terminus" }
export const threadTerminus = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        booleanStepTypePredicate(definition);

        annotation { "Name" : "Thread profile",
                    "Filter" : EntityType.FACE && GeometryType.PLANE && ConstructionObject.NO }
        definition.profile is Query;

        annotation { "Name" : "Helix path",
                    "Filter" : (EntityType.EDGE && ConstructionObject.NO) || (EntityType.BODY && BodyType.WIRE && SketchObject.NO),
                    "MaxNumberOfPicks" : 1 }
        definition.helixPath is Query;

        annotation { "Name" : "Start vertex on helix",
                    "Filter" : EntityType.VERTEX,
                    "MaxNumberOfPicks" : 1 }
        definition.helixVertex is Query;

        annotation { "Name" : "Cylinder face",
                    "Filter" : EntityType.FACE, //&& GeometryType.CYLINDER && ConstructionObject.NO,
                    "MaxNumberOfPicks" : 1 }
        definition.cylinderFace is Query;

        annotation { "Name" : "Taper length" }
        isLength(definition.taperLength, NONNEGATIVE_LENGTH_BOUNDS);

        booleanStepScopePredicate(definition);
    }
    {
        verifyNonemptyQuery(context, definition, "profile", ErrorStringEnum.SWEEP_SELECT_PROFILE);
        verifyNonemptyQuery(context, definition, "helixVertex", ErrorStringEnum.SWEEP_SELECT_PATH);
        // verifyNonemptyQuery(context, definition, "cylinderFace", ErrorStringEnum.EXTRUDE_NO_MERGE_SCOPE);

        // ── Step 1: find the helix edge at the selected vertex ──────────────────────────────────
        // Edges adjacent to the vertex via vertex adjacency = edges that have the vertex as endpoint
        const helixBodyEdges = qOwnedByBody(qEntityFilter(definition.helixPath, EntityType.BODY), EntityType.EDGE);
        const helixDirectEdge = qEntityFilter(definition.helixPath, EntityType.EDGE);
        const allHelixEdges = qUnion([helixBodyEdges, helixDirectEdge]);
        const p0_edge = qIntersection([
                    qAdjacent(definition.helixVertex, AdjacencyType.VERTEX, EntityType.EDGE),
                    allHelixEdges
                ]);
        verifyNonemptyQuery(context, definition, "helixVertex", ErrorStringEnum.SWEEP_SELECT_PATH);

        // Determine whether the vertex is at parameter 0 or 1 on the edge
        const p0 = evVertexPoint(context, { "vertex" : definition.helixVertex });
        const p0_tangentLine = evEdgeTangentLine(context, { "edge" : p0_edge, "parameter" : 0 });
        const p0_parameter = tolerantEquals(p0_tangentLine.origin, p0) ? 0 : 1;
        // If the vertex is at the edge start (param 0) the helix tangent points away from us,
        // so flip the tangent to continue past the endpoint in the correct direction.
        const flip1 = (p0_parameter == 0);

        // ── Step 2: extract G3 curvature data at the helix endpoint ─────────────────────────────
        const curvatureResult = evEdgeCurvature(context, {
                    "edge" : p0_edge,
                    "parameter" : p0_parameter,
                    "arcLengthParameterization" : false
                });
        const kPrimeRaw = evEdgeCurvatureDerivative(context, {
                    "edge" : p0_edge,
                    "parameter" : p0_parameter
                });

        const p0_tangent = flip1 ? -curvatureFrameTangent(curvatureResult) : curvatureFrameTangent(curvatureResult);
        const p0_kPrime = flip1 ? -kPrimeRaw : kPrimeRaw;
        const p0_normal = curvatureFrameNormal(curvatureResult);


        // ── Step 3: surface normal at p0 ────────────────────────────────────────────────────────
        const distResult = evDistance(context, { "side0" : p0, "side1" : definition.cylinderFace });
        const q0 = distResult.sides[1].point;
        const q0_surfaceNormal = evFaceTangentPlane(context, {
                        "face" : definition.cylinderFace,
                        "parameter" : distResult.sides[1].parameter
                    }).normal;

        // ── Step 4: profile depth and p1 ────────────────────────────────────────────────────────
        const profileBox = evBox3d(context, {
                    "topology" : definition.profile,
                    "cSys" : coordSystem(p0, q0_surfaceNormal, p0_tangent),
                    "tight" : true
                });
        const profileDepth = profileBox.maxCorner[0];
        const p1 = p0 + profileDepth * q0_surfaceNormal;

        // ── Step 5: intersection curve on surface ────────────────────────────────────────────────
        // Section plane spanned by q0_surfaceNormal and p0_tangent.
        opPlane(context, id + "sectionPlane", {
                    "plane" : plane(q0, normalize(cross(p0_tangent, q0_surfaceNormal))),
                });
        opIntersectFaces(context, id + "surfaceCurve", {
                    "tools" : qCreatedBy(id + "sectionPlane", EntityType.FACE),
                    "targets" : definition.cylinderFace
                });
        const surfaceCurveEdge = qCreatedBy(id + "surfaceCurve", EntityType.EDGE);
        // debug(context, surfaceCurveEdge);

        // ── Step 6: find p2 at arc length taperLength along the surface curve ──────────────────────
        const q0_parameter = evDistance(context, {
                        "side0" : q0,
                        "side1" : surfaceCurveEdge,
                    }).sides[1].parameter;

        const q0_tangent = evEdgeTangentLine(context, {
                        "edge" : surfaceCurveEdge,
                        "parameter" : q0_parameter,
                    }).direction;

        const sign = dot(q0_tangent, p0_tangent) > 0 ? 1 : -1;
        const arcLength = evLength(context, { "entities" : surfaceCurveEdge });
        const p2 = evEdgeTangentLine(context, {
                        "edge" : surfaceCurveEdge,
                        "parameter" : q0_parameter + sign * definition.taperLength / arcLength,
                    }).origin;

        // ── Step 7: build bridging curve control points (G3 ↔ G0) ───────────────────────────────
        const side1 = {
                    "degree" : 3,
                    "position" : p1,
                    "tangent" : p0_tangent,
                    "curvatureDirection" : p0_normal,
                    "curvature" : curvatureResult.curvature,
                    "normal" : p0_normal,
                    "kPrime" : p0_kPrime
                } as BridgingSideData;
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

        // ── Step 8: sweep the actual taper body and split excess ────────────────────────────────
        const buildTaperBody = function(opId is Id)
            {
                opCreateBSplineCurve(context, opId + "bridgingCurve", { "bSplineCurve" : bCurve });

                opSweep(context, opId + "sweep", {
                            "profiles" : definition.profile,
                            "path" : qCreatedBy(opId + "bridgingCurve", EntityType.EDGE)
                        });

                opSplitPart(context, opId + "split", {
                            "targets" : qBodyType(qCreatedBy(opId + "sweep", EntityType.BODY), BodyType.SOLID),
                            "tool" : definition.cylinderFace,
                            "keepTools" : true,
                            "useTrimmed": false
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
                opDeleteBodies(context, opId + "deletePlane", {
                            "entities" : qCreatedBy(id + "sectionPlane", EntityType.BODY)
                        });
                opDeleteBodies(context, opId + "deleteSurfaceCurve", {
                            "entities" : qCreatedBy(id + "surfaceCurve", EntityType.BODY)
                        });
            };

        buildTaperBody(id);

        processNewBodyIfNeeded(context, id, definition, buildTaperBody);
    },
    { operationType : NewBodyOperationType.NEW });
