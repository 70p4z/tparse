
FROM python:3.10-slim-buster
LABEL version="0.0.0" maintainer="oto@bidule.io"
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get -y update
RUN apt-get -y install python3-pip

RUN pip3 install --upgrade pip 

# install files and requirements
ADD entrypoint.sh /
ADD iobridge /iobridge
ADD *.py /
ADD requirements.txt /

RUN pip3 install --requirement /requirements.txt

# prepare entry point
RUN chmod 755 /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
