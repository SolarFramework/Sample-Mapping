@echo off
:: Download bag of words vocabulary
echo Download bag of word dictionnaries
curl https://github.com/SolarFramework/SolARModuleFBOW/releases/download/fbowVocabulary/fbow_voc.zip -L -o fbow_voc.zip
echo Unzip bag of word dictionnaries
powershell Expand-Archive fbow_voc.zip -DestinationPath .\data\fbow_voc -F
del fbow_voc.zip

curl https://repository.solarframework.org/generic/FbowVoc/popsift_uint8.fbow -L -o ./data/fbow_voc/popsift_uint8.fbow

:: Download AR device captures
echo Download and install AR device captures
curl https://repository.solarframework.org/generic/captures/hololens/bcomLab/loopDesktopA.zip -L -o loopDesktopA.zip
powershell Expand-Archive loopDesktopA.zip -DestinationPath .\data\data_hololens -F
del loopDesktopA.zip

curl https://repository.solarframework.org/generic/captures/hololens/bcomLab/loopDesktopB.zip -L -o loopDesktopB.zip
powershell Expand-Archive loopDesktopB.zip -DestinationPath .\data\data_hololens -F
del loopDesktopB.zip

curl https://repository.solarframework.org/generic/captures/hololens/hololens_calibration.json -L -o .\data\data_hololens\hololens_calibration.json
