FeatureScript 2960;

import(path : "onshape/std/common.fs", version : "2960.0");

export function debug(context is Context, value is EdgeCurvatureResult)
{
    print("debug: EdgeCurvatureResult ");
    println(value);
    addDebugEdgeCurvatureResult(context, value, undefined);
}

export function debug(context is Context, value is EdgeCurvatureResult, color is DebugColor)
{
    print("debug: EdgeCurvatureResult ");
    println(value);
    addDebugEdgeCurvatureResult(context, value, color);
}

function addDebugEdgeCurvatureResult(context is Context, value is EdgeCurvatureResult, color)
{
    const xColor = color == undefined ? DebugColor.RED : color;
    const yColor = color == undefined ? DebugColor.GREEN : color;
    const zColor = color == undefined ? DebugColor.BLUE : color;
    const origin = value.frame.origin;
    const arrowLength = 0.05 * meter;
    const arrowRadius = 0.05 * arrowLength;
    addDebugArrow(context, origin, origin + value.frame.xAxis * arrowLength, arrowRadius, xColor);
    addDebugArrow(context, origin, origin + yAxis(value.frame) * arrowLength, arrowRadius * (2 / 3), yColor);
    addDebugArrow(context, origin, origin + value.frame.zAxis * arrowLength, arrowRadius * 0.5, zColor);

    if (stripUnits(value.curvature) > 1e-10)
    {
        const r = 1 / value.curvature;
        const center = origin + r * value.frame.xAxis;
        const circleColor = color == undefined ? DebugColor.CYAN : color;
        addDebugCircle(context, circle(center, -value.frame.xAxis, yAxis(value.frame), r), circleColor);
        addDebugPoint(context, center, circleColor);
    }
}

export function debug(context is Context, value is Circle)
{
    debug(context, value, DebugColor.RED);
}

export function debug(context is Context, value is Circle, color is DebugColor)
{
    print("debug: Circle ");
    println(value);
    addDebugCircle(context, value, color);
}

function addDebugCircle(context is Context, value is Circle, color is DebugColor)
{
    const circleId = getLastActiveId(context) + "debugCircle";
    startFeature(context, circleId, {});
    try
    {
        const sketch = newSketchOnPlane(context, circleId, { "sketchPlane" : plane(value.coordSystem) });
        skCircle(sketch, "circle", {
                    "center" : vector(0, 0) * meter,
                    "radius" : value.radius
                });
        skSolve(sketch);
        addDebugEntities(context, qBodyType(qCreatedBy(circleId, EntityType.BODY), BodyType.WIRE), color);
    }
    abortFeature(context, circleId);
}
