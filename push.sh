#!/bin/sh

echo "+++++++++++++++++++++++++++++++++++++"
echo "Update option!"
echo "+++++++++++++++++++++++++++++++++++++"

git status

read -r -p "Do you want to push the local Repo to github? [Y/n] " input

case $input in
    [yY][eE][sS]|[yY])
		echo "    "
		echo "---------- Start uploading ---------"
		echo "    "

		git add .
		git commit -m "add note"
		git push origin master

		echo "+++++++++++++++++++++++++++++++++++++"
		echo "Update Finished!"
		echo "+++++++++++++++++++++++++++++++++++++"

		echo "    "
		echo "    "
		;;

    [nN][oO]|[nN])
		echo "---------- Update will not be uploaded! ---------"
       	;;

    *)
		echo "Invalid input..."
		exit 1
		;;
esac


echo "+++++++++++++++++++++++++++++++++++++"
echo "sh exit!"
echo "+++++++++++++++++++++++++++++++++++++"