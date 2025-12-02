# 1. Choose a base image for ARM architecture (e.g., python:3.11-slim-buster or python:3-slim)
# The official Python images are multi-arch and will automatically choose the right one for your Pi
FROM python:3.11-slim

# 2. Set the working directory inside the container
WORKDIR /app

# 3. Copy only the requirements file first for better build caching
# This step only runs if requirements.txt changes, speeding up subsequent builds
COPY requirements.txt .

# 4. Install Python dependencies
# --no-cache-dir reduces the image size
RUN pip install --no-cache-dir -r requirements.txt

# 5. Copy the rest of your application code
COPY . .

# 6. Expose the port your application runs on (if it's a web app, e.g., Flask/Django)
# Replace 5000 with your application's port if different
EXPOSE 5000

# 7. Set the default command to run your application when the container starts
# Replace 'your_script.py' with the name of your main Python file
CMD ["python", "python.py"]