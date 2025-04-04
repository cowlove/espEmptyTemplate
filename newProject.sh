#!/bin/bash
if [ "$1" == "" ]; then
	echo usage "$0" '<new project name>'
	exit
fi

cd `dirname $0`
DIR="${HOME}/src/$1"

mkdir -p "$DIR"
cp make_build_script.sh setup.sh *.ino .gitignore mpub Makefile "${DIR}"
cd "$DIR"
git init
git add .gitignore *
git commit -m "Initial commit"
git config push.autoSetupRemote true
gh repo create --source=. --public
git push

