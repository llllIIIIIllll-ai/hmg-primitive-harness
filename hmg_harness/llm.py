"""Token source: the Anthropic-compatible endpoint from the environment.

The session's endpoint is ``ANTHROPIC_BASE_URL`` (Ollama in this setup) with
``ANTHROPIC_AUTH_TOKEN`` / ``ANTHROPIC_API_KEY``.  Nothing is hard-coded and
no secret is ever printed by this module.
"""
import json
import os
import urllib.error
import urllib.request

BASE = os.environ.get('ANTHROPIC_BASE_URL', 'https://api.anthropic.com').rstrip('/')
MODEL = os.environ.get('PP_LLM_MODEL', os.environ.get('ANTHROPIC_MODEL', 'deepseek-v4.1-flash:1017'))


def _headers():
    h = {'content-type': 'application/json',
         'anthropic-version': '2023-06-01'}
    tok = os.environ.get('ANTHROPIC_AUTH_TOKEN')
    key = os.environ.get('ANTHROPIC_API_KEY')
    if tok:
        h['authorization'] = 'Bearer ' + tok
    if key:
        h['x-api-key'] = key
    return h


def chat(messages, model=None, max_tokens=1024, system=None, tools=None, timeout=120):
    """One Messages-API call.  Returns (text, raw)."""
    body = {'model': model or MODEL, 'max_tokens': int(max_tokens),
            'messages': messages}
    if os.environ.get('PP_LLM_THINKING', '0') != '1':
        # the proxy defaults to a long thinking block that can eat the whole
        # max_tokens budget and return no text at all
        body['thinking'] = {'type': 'disabled'}
    if system:
        body['system'] = system
    if tools:
        body['tools'] = tools
    req = urllib.request.Request(
        BASE + '/v1/messages', data=json.dumps(body).encode(),
        headers=_headers(), method='POST')
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        detail = e.read().decode()[:800]
        raise RuntimeError('LLM HTTP %d: %s' % (e.code, detail)) from None
    text = ''.join(b.get('text', '') for b in raw.get('content', [])
                   if b.get('type') == 'text')
    return text, raw


def ask_json(prompt, schema_hint='', model=None, max_tokens=1024):
    """Ask for a single JSON object and parse it (fenced block tolerated)."""
    sys = ('You reply with ONE JSON object and nothing else. ' + schema_hint)
    text, raw = chat([{'role': 'user', 'content': prompt}], model=model,
                     max_tokens=max_tokens, system=sys)
    t = text.strip()
    if '```' in t:
        t = t.split('```')[1]
        t = t.split('\n', 1)[1] if '\n' in t else t
        t = t.rsplit('```', 1)[0]
    i, j = t.find('{'), t.rfind('}')
    if i < 0 or j < 0:
        raise RuntimeError('no JSON in reply: %r' % text[:300])
    return json.loads(t[i:j + 1]), raw
