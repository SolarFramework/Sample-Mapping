# Download bag of words vocabulary
curl https://github.com/SolarFramework/SolARModuleFBOW/releases/download/fbowVocabulary/fbow_voc.zip -L -o fbow_voc.zip
unzip -o fbow_voc.zip -d ./data
rm fbow_voc.zip

# Download  captures
curl https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household_validation-rgb.avi -L -o ./data/rgbd_dataset_freiburg3_long_office_household_validation-rgb.avi

