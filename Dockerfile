# Base image - Python with Alpine Linux
FROM python:3.9

# Set the working directory inside the container
WORKDIR /app

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        ninja-build \
        libgtk-3-dev \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev
# Install system dependencies
#RUN apk update && apk add --no-cache gcc musl-dev linux-headers

# Copy the application code to the container
COPY . /app

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Expose the application's port
EXPOSE 5000

# Start the Flask application
CMD ["python", "Esp32_hardware_addition.py"]
