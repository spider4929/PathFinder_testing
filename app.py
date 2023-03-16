from flask import Flask
from pathfinder import pathfinder

app = Flask(__name__)

@app.route("/")
def hello():
    return '<h1>PathFinder API running</h1>'

@app.route("/route/<y_orig>,<x_orig>,<y_dest>,<x_dest>/")
def find_path(y_orig, x_orig, y_dest, x_dest):
    y_orig = float(y_orig)
    x_orig = float(x_orig)
    y_dest = float(y_dest)
    x_dest = float(x_dest)

    route = pathfinder([y_orig, x_orig],[y_dest, x_dest])

    return route

if __name__ == "__main__":
  app.run(port=8888)
