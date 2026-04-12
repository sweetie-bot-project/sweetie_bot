import time
import requests
from collections import namedtuple

from typing import Tuple
from typing import Any

__all__ = [
    'Balancer'
]

class BalancerError(RuntimeError):
    def __init__(self, msg, details = ''):
        super(BalancerError, self).__init__(msg)
        self._details = details

    @property
    def details(self):
        return self._details

class Balancer:

    Server = namedtuple('Server', 'name url priority')

    def __init__(self, config, loggers=None, fallback_func=None, postprocess_func=None):
        # get server list
        try:
            self.servers = [ Balancer.Server(server_name, **server_desc) for server_name, server_desc in config['server_choices'].items() ]
        except (TypeError, KeyError, AttributeError):
            raise BalancerError("'server_choices' parameter must present and have the following format { <name> : {'url': <url>, 'priority': <pri> }, ... }")
        self.servers = sorted(self.servers, key = lambda server: server.priority)
        # logger
        if loggers:
            self.log_debug = loggers['debug']
            self.log_warn  = loggers['warn']
            self.log_info  = loggers['info']
        else:
            self.log_debug = lambda x: print(f'Balancer [DEBUG]: {x}')
            self.log_warn  = lambda x: print(f'Balancer [WARN]: {x}')
            self.log_inro  = lambda x: print(f'Balancer [INFO]: {x}')
        # function callbacks
        self.fallback_function = fallback_func
        self.postprocess_function = postprocess_func

        self.log_info(f'servers: {self.servers}')

    def request_available_server(self, decode_json=False, **kwargs):
        # display request
        self.log_debug(f"request: \n\n {kwargs} \n\n")
        # try servers in priority order
        response = None
        for server in self.servers:
            # request server
            try:
                start = time.time()
                response = requests.post(server.url, **kwargs)
                duration = time.time() - start
            except requests.ConnectionError as e:
                 self.log_warn(f'connection error with {server}: {e}')
                 continue
            # check response
            if response.status_code == 200:
                # success
                break
            else:
                self.log_warn(f'{server} request error {response.status_code}: {response.reason}')
                response = None

        # check if response is received
        if response is None:
            # if fallback function is present use it to produce response
            if self.fallback_function is None:
                raise BalancerError('unable to get response: tried all servers.')
            else:
                fallback_response = self.fallback_function(kwargs)
                return fallback_response, 0
        # decode 
        if decode_json:
            try:
                response = response.json()
            except ValueError:
                # TODO: Check behavior when there's html page returned in response to replace some error codes
                raise BalancerError('json: can not decode response', response.content)
        # call postprocessing
        if self.postprocess_function:
            try:
                response = self.postprocess_function(response)
            except Exception as e:
                # TODO: Check behavior when there's html page returned in response to replace some error codes
                raise BalancerError(f'postprocess: got an error during postprocess phase: {e}', response.get('detail', 'Unknown error'))
        # return result
        self.log_debug(f'{server} response: \n\n {response} \n\n')
        return response, duration
