#!/bin/bash

set -e
set -x

sudo apt-get install -y git golang-1.6-go
sudo ln -sf /usr/lib/go-1.6/bin/go  /usr/local/bin/go
git clone -b v2.2.2 https://github.com/github/hub.git;
(cd hub; ./script/build; sudo cp hub /usr/local/bin/)

mkdir -p ~/.config/
echo "github.com:" > ~/.config/hub
echo "- user: k-okada" >> ~/.config/hub
echo "  oauth_token: $GITHUB_ACCESS_TOKEN" >> ~/.config/hub
for file in $CIRCLE_ARTIFACTS/*.{pdf,deb} ; do 
    hub release create -p -a $file -m "$CIRCLE_TAG"$'\n'"Released on `date '+%Y/%m/%d %H:%M:%S'`" $CIRCLE_TAG;
done

