#!/bin/sh

## Detect MAPUPDATE_SERVICE_URL var and use its value 
## to set the Map Update service URL in XML configuration file

if [ -z "$MAPUPDATE_SERVICE_URL" ]
then
    echo "Error: You must define MAPUPDATE_SERVICE_URL env var with the MapUpdate Service URL"
    exit 1 
else
    echo "MAPUPDATE_SERVICE_URL defined: $MAPUPDATE_SERVICE_URL"
fi

echo "Try to replace the MapUpdate Service URL in the XML configuration file..."

sed -i -e "s/MAPUPDATE_SERVICE_URL/$MAPUPDATE_SERVICE_URL/g" /.xpcf/SolARService_Mapping_Multi_NoDrop_properties.xml

echo "XML configuration file ready"

export LD_LIBRARY_PATH=/SolARServiceMappingMultiNoDrop:/SolARServiceMappingMultiNoDrop/modules/

## Start client
cd /SolARServiceMappingMultiNoDrop
./SolARService_Mapping_Multi_NoDrop -m /.xpcf/SolARService_Mapping_Multi_NoDrop_modules.xml -p /.xpcf/SolARService_Mapping_Multi_NoDrop_properties.xml

