FeatureScript 2656;
export import(path : "onshape/std/common.fs", version : "2656.0");
pointPatternIcon::import(path : "9e86ebf436523a29af9bf32f/6603a05cec5be40e8a66a8b3/cd08a2b73e8f36bcd67ffd6b", version : "0194c98edc0b3e878e07a9cc");
pointPatternDescriptionImage::import(path : "b61e162110f17df9506cc2f6", version : "a55cd5e045cc54fb10681608");

annotation {
        "Feature Type Name" : "Point Pattern",
        "Icon" : pointPatternIcon::BLOB_DATA,
        "Description Image" : pointPatternDescriptionImage::BLOB_DATA,
        "Feature Type Description" : "Creates multiple instances of parts, faces, or features at specified points" }
export const pointPattern = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        patternTypePredicate(definition);

        annotation { "Name" : "Reference Point", "Filter" : EntityType.VERTEX || BodyType.MATE_CONNECTOR, "MaxNumberOfPicks" : 1 }
        definition.reference is Query;

        annotation { "Name" : "Locations", "Filter" : EntityType.VERTEX || BodyType.MATE_CONNECTOR }
        definition.locations is Query;

        if (definition.patternType == PatternType.PART)
        {
            booleanPatternScopePredicate(definition);
        }

        if (definition.patternType == PatternType.FEATURE)
        {
            annotation { "Name" : "Apply per instance" }
            definition.fullFeaturePattern is boolean;
        }
    }
    {
        verifyNonemptyQuery(context, definition, "locations", "Select points to pattern");
        definition = adjustPatternDefinitionEntities(context, definition, false);

        // Determine where to pattern
        const locations = evaluateQuery(context, definition.locations);
        verifyPatternSize(context, id, size(locations));

        // Compute the origin
        var origin;
        var originCSys = undefined;
        if (evaluateQuery(context, definition.reference) == [])
        {
            var boxEntities;

            if (definition.patternType == PatternType.PART)
            {
                boxEntities = definition.entities;
            }
            else if (definition.patternType == PatternType.FACE)
            {
                boxEntities = definition.faces;
            }
            else if (definition.patternType == PatternType.FEATURE)
            {
                boxEntities = qCreatedBy(definition.instanceFunction);
            }

            const boxResult = evBox3d(context, { "topology" : boxEntities });
            origin = box3dCenter(boxResult);
        }
        else
        {
            origin = evVertexPoint(context, { "vertex" : definition.reference });
            if (evaluateQuery(context, qBodyType(definition.reference, BodyType.MATE_CONNECTOR)) != [])
            {
                originCSys = evMateConnector(context, { "mateConnector" : definition.reference });
            }
        }

        // Compute the transforms and instance names
        var remainingTransform = getRemainderPatternTransform(context, { "references" : qUnion([getReferencesForRemainderTransform(definition), definition.locations]) });
        var instanceNames = [];
        var transforms = [];
        var patternNumber = 1;
        for (var location in locations)
        {
            var instanceTransform;
            if (originCSys != undefined && evaluateQuery(context, qBodyType(location, BodyType.MATE_CONNECTOR)) != [])
            {
                const locactionCSys = evMateConnector(context, { "mateConnector" : location });
                instanceTransform = toWorld(locactionCSys) * fromWorld(originCSys);
            }
            else
            {
                const point is Vector = evVertexPoint(context, { "vertex" : location });
                instanceTransform = transform(point - origin);
            }
            transforms = append(transforms, instanceTransform);
            instanceNames = append(instanceNames, "" ~ patternNumber);
            patternNumber += 1;
        }
        definition.transforms = transforms;
        definition.instanceNames = instanceNames;
        definition.seed = definition.entities;

        //Pattern
        applyPattern(context, id, definition, remainingTransform);
    });
