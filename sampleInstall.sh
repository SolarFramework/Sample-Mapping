# Download bag of words vocabulary
curl https://github.com/SolarFramework/SolARModuleFBOW/releases/download/fbowVocabulary/fbow_voc.zip -L -o fbow_voc.zip
unzip -o fbow_voc.zip -d ./data
rm fbow_voc.zip

# Download AR device captures
curl https://artifact.b-com.com/solar-generic-local/captures/hololens/bcomLab/loopDesktopA.zip -L -o loopDesktopA.zip
unzip -o loopDesktopA.zip -d ./data
rm loopDesktopA.zip

curl https://artifact.b-com.com/solar-generic-local/captures/hololens/bcomLab/loopDesktopB.zip -L -o loopDesktopB.zip
unzip -o loopDesktopB.zip -d ./data
rm loopDesktopB.zip

curl https://artifact.b-com.com/solar-generic-local/captures/hololens/hololens_calibration.yml -L -o ./data/data_hololens/hololens_calibration.yml

# Download maps
curl https://artifact.b-com.com/solar-generic-local/maps/hololens/bcomLab/loopDesktopA.zip -L -o mapA.zip
unzip -o mapA.zip -d ./data
rm mapA.zip

# Download maps
curl https://artifact.b-com.com/solar-generic-local/maps/hololens/bcomLab/loopDesktopB.zip -L -o mapB.zip
unzip -o mapB.zip -d ./data
rm mapB.zip

# Install required external modules
remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug

