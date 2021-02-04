# Download bag of words vocabulary
curl https://github.com/SolarFramework/SolARModuleFBOW/releases/download/fbowVocabulary/fbow_voc.zip -L -o fbow_voc.zip
mkdir -p data/fbow_voc
unzip -o fbow_voc.zip -d ./data/fbow_voc
rm fbow_voc.zip

# Download AR device captures
curl https://artifact.b-com.com/solar-generic-local/captures/hololens/bcomLab/loopDesktopA.zip -L -o loopDesktopA.zip
unzip -o loopDesktopA.zip -d ./data/data_hololens
rm loopDesktopA.zip

curl https://artifact.b-com.com/solar-generic-local/captures/hololens/bcomLab/loopDesktopB.zip -L -o loopDesktopB.zip
unzip -o loopDesktopB.zip -d ./data/data_hololens
rm loopDesktopB.zip

# Download maps
curl https://artifact.b-com.com/solar-generic-local/maps/hololens/bcomLab/loopDesktopA.zip -L -o mapA.zip
unzip -o mapA.zip -d ./data/map_hololens
rm mapA.zip

curl https://artifact.b-com.com/solar-generic-local/maps/hololens/bcomLab/loopDesktopB.zip -L -o mapB.zip
unzip -o mapB.zip -d ./data/map_hololens
rm mapB.zip

