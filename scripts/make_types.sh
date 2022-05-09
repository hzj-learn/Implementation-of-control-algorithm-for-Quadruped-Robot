#!/bin/bash


GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN} Starting LCM type generation...${NC}"     #在终端告诉你，正在征程LCM类型

cd ../lcm-types
# Clean，清除指令，删除上次编译产生的中间文件
rm */*.jar
rm */*.java
rm */*.hpp
rm */*.class
rm */*.py
rm */*.pyc

# Make，编译程序的时候会运行下面这些指令进行编译
lcm-gen -jxp *.lcm
cp /usr/local/share/java/lcm.jar .
javac -cp lcm.jar */*.java
jar cf my_types.jar */*.class
mkdir -p java
mv my_types.jar java
mv lcm.jar java
mkdir -p cpp
mv *.hpp cpp

mkdir -p python
mv *.py python

FILES=$(ls */*.class)
echo ${FILES} > file_list.txt


echo -e "${GREEN} Done with LCM type generation${NC}"       #终端告诉你，LCM的类型已经编译生成完毕了
