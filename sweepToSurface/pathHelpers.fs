FeatureScript 2960;

import(path : "onshape/std/common.fs", version : "2960.0");

/**
 * Like `evDistance` but accepts a `Path` for `side0` or `side1`, returning an arc-length
 * parameter (0..1 along the full path) instead of an edge-local parameter.
 * Can be used in conjection with evPathTangentLines.
 * Adapted from: https://forum.onshape.com/discussion/comment/109246/#Comment_109246
 */
export function evDistancePath(context is Context, definition is map) returns map
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
export function pathParameterFromEdge(context is Context, path is Path, edge is Query, edgeParam is number) returns number
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

