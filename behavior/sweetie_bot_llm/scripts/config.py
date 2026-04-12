from os import environ 

class Config(object):
    DEBUG = False

class ProductionConfig(Config):
    # put it ot .env
    API_KEY=environ.get("API_KEY")
    HOST="0.0.0.0"
    PORT="5011"

class DevelopmentConfig(Config):
    API_KEY="test_api_key"
    DEBUG = True
