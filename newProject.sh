#!/bin/bash
if [ "$1" == "" ]; then
	echo usage "$0" '<new project name>'
	exit
fi

cd `dirname $0`
DIR="${HOME}/src/$1"
SKETCH=`basename $1`

mkdir -p "$DIR/bin"
cp *.ino "${DIR}/${SKETCH}.ino"
cp setup.sh .gitignore mpub Makefile "${DIR}"
cp ./bin/cli-parser.py "${DIR}/bin/"
cd "$DIR"
git init
git add .gitignore * ./bin/cli-parser.py
git commit -m "Initial commit"
git config push.autoSetupRemote true
gh repo create --source=. --public
git push

