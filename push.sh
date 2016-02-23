echo "-------------------------------------->"
echo "CLEANING..."
cd KinectCoreN
make clean
cd ..
echo "-------------------------------------->"
echo "COMMITING..."
git commit -a -m $1
echo "-------------------------------------->"
echo "PUSHING..."
git push
