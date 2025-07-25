#!/bin/zsh

set -e

echo "Copying assets..."

cp -r ./assets/* ./curobo/src/curobo/content/assets/
cp -r ./configs/* ./curobo/src/curobo/content/configs/

echo "Copy assets complete."
