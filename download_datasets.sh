mkdir datasets/Middlebury -p
cd datasets/Middlebury/
wget https://www.dropbox.com/s/fn8siy5muak3of3/official_train.txt -P MiddEval3/
wget https://vision.middlebury.edu/stereo/submit3/zip/MiddEval3-data-Q.zip
unzip MiddEval3-data-Q.zip

rm *.zip
cd ../..

mkdir datasets/ETH3D/two_view_testing -p
cd datasets/ETH3D/two_view_testing
wget https://www.eth3d.net/data/two_view_test.7z
echo "Unzipping two_view_test.7z using p7zip (installed from environment.yaml)"
7za x two_view_test.7z
cd ../../..