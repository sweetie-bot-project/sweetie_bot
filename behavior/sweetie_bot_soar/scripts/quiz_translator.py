#!/usr/bin/env python3
import yaml, sys

class QuizParseError(RuntimeError):
    def __init__(self, question, msg):
        super(QuizParseError, self).__init__("Processing state question %s: %s" % (question, msg))

def get_attribute(desc, attribute):
    value = desc.get(attribute)
    if value is not None:
        return '^%s |%s|' % (attribute, value)
    else:
        return ''

if len(sys.argv) != 2:
    print("Usage: %s QUIZ_FILE" % sys.argv[0])
    print("")
    print("Translate QUIZ_FILE to soar rules and print them to standart output. QUIZ_FILE should contain quiz description in YAML format.")
    sys.exit(1)

with open(sys.argv[1]) as f:
    quiz_desc = yaml.safe_load(f)

question_number = 0
for question_desc in quiz_desc:
    # check state
    # header
    print("sp {quiz*elaborate*question-%d" % question_number)
    # rule lhs
    print("\t(state <s> ^top-state 1 ^mem.quiz <quiz>)")
    # separator
    print("-->")
    # rule lhs: get all elements
    try:
        question = question_desc['question']
        statement = question_desc['statement']
        answers = question_desc['answers']
    except KeyError as e:
        raise QuizParseError(question_number, "'question', 'statement' and 'answers' attribute must present in question description.")
   # consruct question description
    print("\t(<quiz> ^question <q>)")
    print("\t(<q> ^question |%s|)" % question)
    print("\t(<q> ^statement |%s|)" % statement)
    # parse answers
    answer_id = 0
    answers_rhs = ''
    answers_ids = ''
    for answer, answer_desc in answers.items():
        wme_id = "<a%d>" % answer_id
        # collect rhs
        result = answer_desc['result']
        if result not in ('correct-answer', 'incorrect-answer', 'skip-question', 'repeat-question'):
            raise QuizParseError(question_number, "Answer 'result' must be 'correct-answer', 'incorrect-answer', 'skip-question' or 'repeat-question'.")
        answers_rhs +=  "\t(%s ^answer-topic %s ^reaction |%s| ^result %s)\n" % (wme_id, answer, answer_desc['reaction'], result)
        # append segement wme id to list
        answers_ids += ' ' + wme_id
        # increase segment id
        answer_id += 1
    # output answer segemnts
    print("\t(<q> ^answer %s)" % answers_ids)
    print(answers_rhs[:-1])
    # finish rule
    print("}\n")

	# increase question number
    question_number += 1


