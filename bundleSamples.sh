set -e

Version="1.0.0"

if [ -z "$1" ]
then
   filename="SolARSample_Mapping_$Version"
else
   filename=$1
fi

CONFIG_FILES=`find . -not \( -path "./deploy/bin" -prune \) -type f \( -name "SolARSample*_conf.xml" -or -name "SolARPipelineTest*_conf.xml" \)`

echo "**** Displaying profiles"
echo $(remaken profile display)
echo $(conan profile show default)
echo "**** Install dependencies locally"

remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug

echo "**** Bundle dependencies in bin folder"
for file in $CONFIG_FILES
do
   echo "install dependencies for config file: $file"
   remaken bundleXpcf --recurse -d ./deploy/bin/x86_64/shared/release -s modules $file
   remaken bundleXpcf --recurse -d ./deploy/bin/x86_64/shared/debug -s modules -c debug $file
done

cp ./runFromBundle.sh ./run.sh
mv ./run.sh ./deploy/bin/x86_64/shared/release/
cp ./runFromBundle.sh ./run.sh
mv ./run.sh ./deploy/bin/x86_64/shared/debug


zip --symlinks -r "./deploy/${filename}_release.zip" ./deploy/bin/x86_64/shared/release ./README.md ./installData.sh ./LICENSE
zip --symlinks -r "./deploy/${filename}_debug.zip" ./deploy/bin/x86_64/shared/debug ./README.md ./installData.sh ./LICENSE 
