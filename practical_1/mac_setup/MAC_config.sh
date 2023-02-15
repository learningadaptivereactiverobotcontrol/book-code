#!/bin/bash

echo "Installing brew”
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

echo "Installing llvm”
brew install llvm

echo "File(s) are ready to use, "
echo "Try to run setup_code_ch3  "

mex -setup
read terminate
