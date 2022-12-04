#!/bin/bash
IMAGE_FOLDER_DIR=write_pics


if [ -d ${IMAGE_FOLDER_DIR} ]
then
	rm -rf ${IMAGE_FOLDER_DIR}/*
else
	mkdir ${IMAGE_FOLDER_DIR}
fi

cmake .
make

