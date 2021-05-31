#!/bin/bash

# Test number of parameters
if [[ $# -ne 3 ]]; then
	echo "Invalid parameters!"
	echo "Correct syntaxe is: runRemoteDebug.sh RemoteServerName -f configuration_file.xml"
	exit
fi

export REMAKENROOT=$HOME/.remaken/packages/linux-gcc
export PKG_CONFIG_PATH=/home/linuxbrew/.linuxbrew/opt/openssl/lib/pkgconfig:/home/linuxbrew/.linuxbrew/lib/pkgconfig/

# Set remaken package root path
export XPCF_MODULE_ROOT=~/.remaken/packages/linux-gcc
echo "XPCF_MODULE_ROOT=$XPCF_MODULE_ROOT"

echo "Configuration file = $3"

# Include dependencies path to ld_library_path
ld_library_path="./"
if [ -f "$PWD/$3" ]; then
	for modulePath in $(grep -o "\$XPCF_MODULE_ROOT.*lib" $3)
	do
	   modulePath=${modulePath/"\$XPCF_MODULE_ROOT"/${XPCF_MODULE_ROOT}}
	   if ! [[ $ld_library_path =~ "$modulePath/x86_64/shared/debug" ]]
	   then
		  ld_library_path=$ld_library_path:$modulePath/x86_64/shared/debug
	   fi 
	done
fi

# Add brew library path
ld_library_path=$ld_library_path:/home/linuxbrew/.linuxbrew/lib

echo "LD_LIBRARY_PATH=$ld_library_path $1 $2 $3"

# Run program
LD_LIBRARY_PATH=$ld_library_path $1 $2 $3



