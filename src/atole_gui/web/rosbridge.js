// Cliente mínimo del protocolo de rosbridge 2.x (JSON por WebSocket), sin dependencias.
// Suscripciones (se restablecen al reconectar), servicios y acciones de ROS 2 con feedback y cancelación.
'use strict';

class Ros {
  constructor(url) {
    this.url = url;
    this.nextId = 1;
    this.subs = new Map();       // topic -> {type, opts, callbacks: Set}
    this.pending = new Map();    // id -> {resolve, reject, onFeedback}
    this.statusListeners = new Set();
    this.connected = false;
    this._connect();
  }

  onStatus(fn) { this.statusListeners.add(fn); fn(this.connected); }

  _id(prefix) { return `${prefix}:${this.nextId++}`; }

  _send(msg) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) { this.ws.send(JSON.stringify(msg)); return true; }
    return false;
  }

  _setStatus(ok) {
    this.connected = ok;
    this.statusListeners.forEach(fn => fn(ok));
  }

  _connect() {
    this.ws = new WebSocket(this.url);
    this.ws.onopen = () => {
      this._setStatus(true);
      for (const [topic, s] of this.subs) this._subscribe(topic, s);
    };
    this.ws.onclose = () => {
      const was = this.connected;
      this._setStatus(false);
      for (const [, p] of this.pending) p.reject(new Error('conexión con rosbridge perdida'));
      this.pending.clear();
      setTimeout(() => this._connect(), was ? 500 : 2000);
    };
    this.ws.onerror = () => {};
    this.ws.onmessage = ev => this._onMessage(JSON.parse(ev.data));
  }

  _onMessage(m) {
    if (m.op === 'publish') {
      const s = this.subs.get(m.topic);
      if (s) s.callbacks.forEach(cb => { try { cb(m.msg); } catch (e) { console.error(m.topic, e); } });
      return;
    }
    const p = this.pending.get(m.id);
    if (!p) return;
    if (m.op === 'service_response') {
      this.pending.delete(m.id);
      m.result ? p.resolve(m.values) : p.reject(new Error(typeof m.values === 'string' ? m.values : 'el servicio falló'));
    } else if (m.op === 'action_feedback') {
      if (p.onFeedback) p.onFeedback(m.values);
    } else if (m.op === 'action_result') {
      this.pending.delete(m.id);
      m.result ? p.resolve({ result: m.values, status: m.status })
               : p.reject(new Error(typeof m.values === 'string' ? m.values : 'la acción falló'));
    }
  }

  _subscribe(topic, s) {
    this._send({ op: 'subscribe', id: this._id('sub'), topic, type: s.type,
                 throttle_rate: s.opts.throttle || 0, queue_length: 1 });
  }

  subscribe(topic, type, callback, opts = {}) {
    let s = this.subs.get(topic);
    if (!s) {
      s = { type, opts, callbacks: new Set() };
      this.subs.set(topic, s);
      if (this.connected) this._subscribe(topic, s);
    }
    s.callbacks.add(callback);
    return () => {
      s.callbacks.delete(callback);
      if (!s.callbacks.size) { this.subs.delete(topic); this._send({ op: 'unsubscribe', topic }); }
    };
  }

  call(service, type, args = {}, timeoutMs = 60000) {
    return new Promise((resolve, reject) => {
      const id = this._id('srv');
      this.pending.set(id, { resolve, reject });
      if (!this._send({ op: 'call_service', id, service, type, args })) {
        this.pending.delete(id);
        return reject(new Error('sin conexión con rosbridge'));
      }
      setTimeout(() => { if (this.pending.delete(id)) reject(new Error(`${service}: sin respuesta`)); }, timeoutMs);
    });
  }

  // Devuelve {done: Promise<{result, status}>, cancel()}.
  action(action, type, args = {}, onFeedback = null) {
    const id = this._id('act');
    const done = new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject, onFeedback });
      if (!this._send({ op: 'send_action_goal', id, action, action_type: type, args, feedback: !!onFeedback })) {
        this.pending.delete(id);
        reject(new Error('sin conexión con rosbridge'));
      }
    });
    return { done, cancel: () => this._send({ op: 'cancel_action_goal', id, action }) };
  }
}
