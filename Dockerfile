FROM duckietown/aido2-base-python3:z2

WORKDIR /project

COPY requirements.txt .

RUN pip install -r requirements.txt && rm -rf /root/.cache

COPY . .

ENTRYPOINT ["python3", "solution.py"]