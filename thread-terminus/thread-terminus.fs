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
                    "Filter" : EntityType.FACE && GeometryType.CYLINDER && ConstructionObject.NO,
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
        const edgeAtVertex = qIntersection([
                    qAdjacent(definition.helixVertex, AdjacencyType.VERTEX, EntityType.EDGE),
                    allHelixEdges
                ]);
        verifyNonemptyQuery(context, definition, "helixVertex", ErrorStringEnum.SWEEP_SELECT_PATH);

        // Determine whether the vertex is at parameter 0 or 1 on the edge
        const p0 = evVertexPoint(context, { "vertex" : definition.helixVertex });
        const edgeLine0 = evEdgeTangentLine(context, { "edge" : edgeAtVertex, "parameter" : 0 });
        const helixParam = tolerantEquals(edgeLine0.origin, p0) ? 0 : 1;
        // If the vertex is at the edge start (param 0) the helix tangent points away from us,
        // so flip the tangent to continue past the endpoint in the correct direction.
        const flip1 = (helixParam == 0);

        // ── Step 2: extract G3 curvature data at the helix endpoint ─────────────────────────────
        const curvatureResult = evEdgeCurvature(context, {
                    "edge" : edgeAtVertex,
                    "parameter" : helixParam,
                    "arcLengthParameterization" : false
                });
        const kPrimeRaw = evEdgeCurvatureDerivative(context, {
                    "edge" : edgeAtVertex,
                    "parameter" : helixParam
                });

        const tangent0 = flip1 ? -curvatureFrameTangent(curvatureResult) : curvatureFrameTangent(curvatureResult);
        const kPrime0 = flip1 ? -kPrimeRaw : kPrimeRaw;
        const normal0 = curvatureFrameNormal(curvatureResult);


        // ── Step 3: cylinder geometry ────────────────────────────────────────────────────────────
        const cylinderDef = evSurfaceDefinition(context, { "face" : definition.cylinderFace });
        if (!(cylinderDef is Cylinder))
            throw regenError("Selected face must be cylindrical.", ["cylinderFace"]);
        const cylinderAxisDir = cylinderDef.coordSystem.zAxis;
        const cylinderAxisOrigin = cylinderDef.coordSystem.origin;
        const cylinderRadius = cylinderDef.radius;

        // XXX
        // ── Step 4: conservative profile depth from bounding box diagonal ────────────────────────

        // ── Step 5: compute bridging curve endpoint p2 ──────────────────────────────────────────
        // p2 follows the helix direction for taperLength arc, then sits profileDepth inside the
        // cylinder so the swept profile fully crosses the cylinder surface before the split.
        const p0_axialProjection = cylinderAxisOrigin + dot(p0 - cylinderAxisOrigin, cylinderAxisDir) * cylinderAxisDir;
        const p0_radial = p0 - p0_axialProjection;
        const p0_radialDir = p0_radial / norm(p0_radial);
        const p0_circularDir = cross(cylinderAxisDir, p0_radialDir);

        const profileBox = evBox3d(context, {
            "topology" : definition.profile,
            "cSys" : coordSystem(p0, p0_radialDir, cylinderAxisDir),
            "tight" : true
        });
        const profileDepth = profileBox.maxCorner[0];
        const p1 = p0 + profileDepth * p0_radialDir;

        const axialAdvance = definition.taperLength * dot(tangent0, cylinderAxisDir);
        const circularAdvanceAngle = definition.taperLength * dot(tangent0, p0_circularDir) / norm(p0_radial) * radian;
        const p2_radialDir = rotationMatrix3d(cylinderAxisDir, circularAdvanceAngle) * p0_radialDir;
        const p2 = p0_axialProjection + axialAdvance * cylinderAxisDir + cylinderRadius * p2_radialDir;

        const side1 = {
                    "degree" : 3,
                    "position" : p1,
                    "tangent" : tangent0,
                    "curvatureDirection" : normal0,
                    "curvature" : curvatureResult.curvature,
                    "normal" : normal0,
                    "kPrime" : kPrime0
                } as BridgingSideData;
        const side2 = {
                    "degree" : 0,
                    "position" : p2
                } as BridgingSideData;

        // ── Step 6: build bridging curve control points (G3 ↔ G0) ───────────────────────────────
        const controlPoints = computeBridgingControlPoints(context, side1, side2);
        const bCurve = bSplineCurve({
                    "degree" : size(controlPoints) - 1,
                    "isPeriodic" : false,
                    "controlPoints" : controlPoints
                });

        // ── Step 7: sweep the actual taper body and split excess ────────────────────────────────
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
                            "keepTools" : true
                        });

                // opSplitPart does not create new bodies, but rather modify the existing one, so
                // qCreatedBy(opId + "split", EntityType.BODY) would be empty
                const qKeep = qCreatedBy(opId + "sweep", EntityType.BODY)->qClosestTo(p1);
                const qDelete = qCreatedBy(opId + "sweep", EntityType.BODY)->qSubtraction(qKeep);
                opDeleteBodies(context, opId + "deleteExcess", {
                            "entities" : qDelete
                        });
                opDeleteBodies(context, opId + "deleteCurve", {
                            "entities" : qCreatedBy(opId + "bridgingCurve", EntityType.BODY)
                        });
            };

        buildTaperBody(id);

        processNewBodyIfNeeded(context, id, definition, buildTaperBody);
    },
    { operationType : NewBodyOperationType.NEW });
