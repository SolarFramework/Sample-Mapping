# Download bag of words vocabulary
curl https://github.com/SolarFramework/SolARModuleFBOW/releases/download/fbowVocabulary/fbow_voc.zip -L -o fbow_voc.zip
mkdir -p data/fbow_voc
unzip -o fbow_voc.zip -d ./data/fbow_voc
rm fbow_voc.zip

curl https://artifact.b-com.com/solar-generic-local/FbowVoc/popsift_uint8.fbow -L -o ./data/fbow_voc/popsift_uint8.fbow

# Download AR device captures
curl https://artifact.b-com.com/solar-generic-local/captures/hololens/bcomLab/loopDesktopA.zip -L -o loopDesktopA.zip
unzip -o loopDesktopA.zip -d ./data/data_hololens
rm loopDesktopA.zip

curl https://artifact.b-com.com/solar-generic-local/captures/hololens/bcomLab/loopDesktopB.zip -L -o loopDesktopB.zip
unzip -o loopDesktopB.zip -d ./data/data_hololens
rm loopDesktopB.zip

# Download calibration file
echo Download calibration file
curl https://artifact.b-com.com/solar-generic-local/captures/hololens/hololens_calibration.json -L -o ./data/data_hololens/hololens_calibration.json
