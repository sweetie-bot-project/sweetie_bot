# conftest.py
#from utils import screenfetch
from _pytest.terminal import TerminalReporter

def pytest_configure(config):
    terminal = config.pluginmanager.getplugin('terminal')

    class QuietReporter(terminal.TerminalReporter):

        def short_test_summary(self):
            # your own impl goes here, for example:
            self.write_sep("=", "sweetie bot self test summary info")
            #print(self.stats.get("passed", []))
            reps = self.stats.get("passed", [])
            for rep in reps:
                #print(dir(rep))
                self.write_line(f"green\t{rep.nodeid} {rep.capstdout}")
            reps = self.stats.get("failed", [])
            for rep in reps:
                #print(dir(rep))
                self.write_line(f"red\t{rep.nodeid} {rep.capstdout}")
            reps = self.stats.get("warnings", [])
            for rep in reps:
                print(dir(rep))
                self.write_line(f"yellow\t{rep.nodeid} {rep.message}")

        @property
        def verbosity(self):
            return 0

        @property
        def showlongtestinfo(self):
            return False

        @property
        def showfspath(self):
            return False

        @property
        def rootdir(self):
            return ''

        @property
        def invocation_dir(self):
            return ''


    terminal.TerminalReporter = QuietReporter
    
#def pytest_terminal_summary(terminalreporter, exitstatus, config):
    #print(dir(terminalreporter))
    #print(dir(terminalreporter.config))
    #terminalreporter.config.invocation_dir=''
    #terminalreporter.config.rootdir=''
    #print(terminalreporter.config.invocation_dir)
    #print(terminalreporter.config.rootdir)
    #print(terminalreporter.summary_failures().replace(terminalreporter.config.rootdir, ''))
    #terminalreporter.summary_failures()
    #.replace(terminalreporter.config.rootdir, ''))
    #terminalreporter.ensure_newline()
    #terminalreporter.write(screenfetch())
#    pass
