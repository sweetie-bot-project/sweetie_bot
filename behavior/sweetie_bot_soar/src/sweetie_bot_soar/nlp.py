import spacy

class SpacyInstance:
    _spacy_models = {}
    def __new__(cls, model, *args, **kwargs):
        # check if correspondin model is loaded
        instance = cls._spacy_models.get(model)
        if instance is None:
            # load model
            instance = spacy.load(model, *args, **kwargs)
            cls._spacy_models[model] = instance
        # return spacy model
        return instance

