
FROM python:3.9-slim

# 1. Install System Dependencies for OpenCV & Networking
# libgl1-mesa-glx is REQUIRED for cv2.
# libglib2.0-0 is often needed too.
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    procps \
    && rm -rf /var/lib/apt/lists/*

# 2. Setup Workdir
WORKDIR /app

# 3. Install Python Dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# 4. Copy Code
COPY . .

# 5. Expose Port (Railway uses $PORT)
ENV PORT=8000
EXPOSE $PORT

# 6. Run Command (matches Procfile)
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
