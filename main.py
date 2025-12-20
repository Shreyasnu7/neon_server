from fastapi import FastAPI
from control.arm import execute as arm_drone
from control.takeoff import execute as takeoff_drone
from control.land import execute as land_drone

app = FastAPI(title="Drone Control Server")

@app.get("/")
def root():
    return {"status": "Drone server running"}

@app.post("/arm")
def arm():
    return arm_drone()

@app.post("/takeoff")
def takeoff(altitude: float = 3):
    return takeoff_drone(altitude)

@app.post("/land")
def land():
    return land_drone()
