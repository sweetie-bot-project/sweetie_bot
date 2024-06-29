import time
import requests

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

    def __init__(self, config, loggers=None, fallback_func=None, postprocess_func=None):
        self.server_choices = {} if 'server_choices' not in config else config['server_choices']
        if loggers:
            self.log_debug = loggers['debug']
            self.log_warn  = loggers['warn']
            self.log_info  = loggers['info']
        else:
            self.log_debug = lambda x: print(f'Balancer [DEBUG]: {x}')
            self.log_warn  = lambda x: print(f'Balancer [WARN]: {x}')
            self.log_inro  = lambda x: print(f'Balancer [INFO]: {x}')
            
        self.fallback_function = fallback_func
        self.postprocess_function = postprocess_func

        self.log_info(f'servers: {self.server_choices}')

    def request_available_server(self, decode_json=False, **kwargs):
        # display request
        if 'json' in kwargs:
            self.log_debug(f"request: \n\n {kwargs['json']} \n\n")
        else:
            self.log_warning(f"request: \n\n No json provided! \n\n")

        LOWEST_PRIORITY = 100
        def priority_sorting(server_item: Tuple[str, Any]) -> int:
            return server_item[1].get('priority', LOWEST_PRIORITY)
            
        response = None
        server_name, server_url = None, None
        for name, server in sorted(self.server_choices.items(), key=priority_sorting):
            try:
                start = time.time()
                response = requests.post(server['url'], **kwargs)
                duration = time.time() - start

                if response.status_code == 200:
                    server_name, server_url = name, server['url']
                    break
                else:
                    self.log_warn(f'server {name} ({server_url}) request error {response.status_code}: {response.reason}')
                    continue # try the next server

            except requests.ConnectionError as e:
                self.log_warn(f'connection error with server {name} ({server_url}): {e}')

        # check if response is received
        if response is not None and response.status_code != 200:
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

        if self.postprocess_function:
            try:
                response = self.postprocess_function(response)
            except Exception as e:
                # TODO: Check behavior when there's html page returned in response to replace some error codes
                raise BalancerError(f'postprocess: got an error during postprocess phase: {e}', response.get('detail', 'Unknown error'))

        self.log_debug(f'server {server_name} response: \n\n {response} \n\n')

        return response, duration
