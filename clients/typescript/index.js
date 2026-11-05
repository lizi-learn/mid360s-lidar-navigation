/** No connection is made by constructing this client. Mutations are never retried. */
export class NavigationError extends Error {
  constructor(code, message, status = 0, details = {}) {
    super(message); this.name = 'NavigationError'; this.code = code; this.status = status; this.details = details;
  }
}
export class NavigationClient {
  constructor(baseURL = 'http://127.0.0.1:18940', {apiKey, timeout = 20000} = {}) {
    const url = new URL(baseURL);
    if (!['http:', 'https:'].includes(url.protocol) || url.username || url.password) throw new TypeError('Expected HTTP(S) URL without credentials');
    this.base = baseURL.replace(/\/$/, ''); this.apiKey = apiKey; this.timeout = timeout;
  }
  async request(method, path, data, key) {
    const controller = new AbortController(), timer = setTimeout(() => controller.abort(), this.timeout);
    try {
      const headers = {Accept:'application/json'};
      if (this.apiKey) headers.Authorization = `Bearer ${this.apiKey}`;
      if (data !== undefined) headers['Content-Type'] = 'application/json';
      if (key) headers['Idempotency-Key'] = key;
      const response = await fetch(this.base + path, {method, headers, redirect:'error', signal:controller.signal, ...(data !== undefined ? {body:JSON.stringify(data)} : {})});
      const reader = response.body?.getReader(), chunks = []; let size = 0;
      if (reader) { try { while (true) { const {done,value} = await reader.read(); if (done) break;
        size += value.byteLength;
        if (size > 2000000) { await reader.cancel(); throw new NavigationError('RESPONSE_TOO_LARGE','Response limit exceeded',502); }
        chunks.push(value);
      } } finally { reader.releaseLock(); } }
      const bytes = new Uint8Array(size); let offset = 0;
      for (const chunk of chunks) { bytes.set(chunk,offset); offset += chunk.byteLength; }
      let text;
      try { text = new TextDecoder('utf-8',{fatal:true}).decode(bytes); }
      catch { throw new NavigationError(method === 'GET' ? 'INVALID_RESPONSE' : 'UPSTREAM_UNCERTAIN','Invalid UTF-8 reply',502); }
      if (method !== 'GET' && response.status >= 500) throw new NavigationError('UPSTREAM_UNCERTAIN','Server error after mutation; outcome unknown',503);
      let result;
      try { result = JSON.parse(text); }
      catch { throw new NavigationError(method === 'GET' ? 'INVALID_RESPONSE' : 'UPSTREAM_UNCERTAIN','Invalid JSON reply; mutations are not replayed',502); }
      if (!response.ok) throw new NavigationError(result.error?.code ?? 'HTTP_ERROR', result.error?.message ?? 'Request rejected', response.status, result.error?.details);
      return result;
    } catch (error) {
      if (error instanceof NavigationError) throw error;
      throw new NavigationError(method === 'GET' ? 'UPSTREAM_UNAVAILABLE' : 'UPSTREAM_UNCERTAIN','Request outcome unavailable; no automatic retry',503);
    } finally { clearTimeout(timer); }
  }
  capabilities() { return this.request('GET','/v1/capabilities'); }
  status() { return this.request('GET','/v1/status'); }
  maps() { return this.request('GET','/v1/maps'); }
  markers() { return this.request('GET','/v1/markers'); }
  routes() { return this.request('GET','/v1/routes'); }
  move(target, {idempotencyKey} = {}) { return this.request('POST','/v1/tasks',{kind:'move',target},idempotencyKey); }
  visit(marker, {idempotencyKey} = {}) { return this.request('POST','/v1/tasks',{kind:'visit',marker},idempotencyKey); }
  route(routeId, {idempotencyKey} = {}) { return this.request('POST','/v1/tasks',{kind:'route',route_id:routeId},idempotencyKey); }
  forward(meters, {idempotencyKey} = {}) { return this.request('POST','/v1/tasks',{kind:'forward',meters},idempotencyKey); }
  rotate(radians, {idempotencyKey} = {}) { return this.request('POST','/v1/tasks',{kind:'rotate',radians},idempotencyKey); }
  tour(markers, {idempotencyKey} = {}) { return this.request('POST','/v1/tasks',{kind:'tour',markers},idempotencyKey); }
  task(id) { return this.request('GET',`/v1/tasks/${encodeURIComponent(id)}`); }
  control(id, operation) { return this.request('POST',`/v1/tasks/${encodeURIComponent(id)}/${operation}`,{}); }
  cancel(id) { return this.control(id,'cancel'); }
  pause(id) { return this.control(id,'pause'); }
  resume(id) { return this.control(id,'resume'); }
  continueVisit(id) { return this.control(id,'continue'); }
  stop() { return this.request('POST','/v1/stop',{}); }
  saveMarker(marker) { return this.request('POST','/v1/markers',marker); }
  deleteMarker(name) { return this.request('DELETE',`/v1/markers/${encodeURIComponent(name)}`); }
  async wait(id, {timeout = 120000, interval = 500, signal} = {}) {
    const deadline = Date.now() + timeout;
    while (true) {
      if (signal?.aborted) throw new NavigationError('WAIT_ABORTED','Client wait aborted; navigation has NOT been cancelled',408,{task_id:id});
      const task = await this.task(id);
      if (['succeeded','failed','cancelled','interrupted','unknown'].includes(task.state)) return task;
      if (Date.now() >= deadline) throw new NavigationError('WAIT_TIMEOUT','Wait timed out; navigation has NOT been cancelled',408,{task_id:id});
      await new Promise(resolve => setTimeout(resolve, Math.max(0,Math.min(interval,deadline-Date.now()))));
    }
  }
}
