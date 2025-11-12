from flask import Blueprint, render_template

auth_blueprint = Blueprint("auth", __name__)

@auth_blueprint.route("/login")
def login():
    return render_template("login.html")

@auth_blueprint.route("/register")
def register():
    return render_template("register.html")
