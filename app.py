from flask import Flask, request
from pathfinder import pathfinder

app = Flask(__name__)


@app.route("/")
def hello():
    return '<h1>PathFinder API running</h1>'


@app.route("/route/")
def find_path():
    args = request.args

    y_orig = float(args['y_orig'])
    x_orig = float(args['x_orig'])
    y_dest = float(args['y_dest'])
    x_dest = float(args['x_dest'])

    try:
        if args['adjust']:
            adjust = False
    except:
        adjust = True

    route = pathfinder([y_orig, x_orig], [y_dest, x_dest], adjust)

    return route


if __name__ == "__main__":
    app.run(port=8888)
