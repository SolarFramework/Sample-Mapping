
Version="0.10.0"

if [ -z "$1" ]
then
   filename="SolAR_Mapping_$Version"
else
   filename=$1
fi

CONFIG_FILES=`find . -not \( -path "./bin" -prune \) -type f \( -name "SolARSample*_conf.xml" -or -name "SolARPipelineTest*_conf.xml" \)`

# Update configuration files by replacing win-cl-1.1 by linux in module paths
echo "**** Update module path in configuration file (win-cl-14.1 -> linux-gcc)"
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/bin/Debug/*_conf.xml
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/bin/Release/*_conf.xml
sed -i 's/win-cl-14.1/linux-gcc/' $CONFIG_FILES

echo "**** Install dependencies locally"
remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug

echo "**** Bundle dependencies in bin folder"
for file in $CONFIG_FILES
do
   echo "install dependencies for config file: $file"
   remaken bundleXpcf $file -d ./bin/Release -s modules
   remaken bundleXpcf $file -d ./bin/Debug -s modules -c debug
done

cp ./runFromBundle.sh ./run.sh
mv ./run.sh ./bin/Release/
cp ./runFromBundle.sh ./run.sh
mv ./run.sh ./bin/Debug


zip --symlinks -r "./bin/${filename}_release.zip" ./bin/Release ./README.md ./installData.sh ./LICENSE
zip --symlinks -r "./bin/${filename}_debug.zip" ./bin/Debug ./README.md ./installData.sh ./LICENSE 
