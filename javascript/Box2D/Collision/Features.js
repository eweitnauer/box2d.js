var Features = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
Features.prototype.__constructor = function(){}
Features.prototype.__varz = function(){
}
// static attributes
// static methods
// attributes
// methods
Features.prototype.get = function () {
		return _referenceEdge;
	}
Features.prototype.set = function (value) {
		_referenceEdge = value;
		_m_id._key = (_m_id._key & 0xffffff00) | (_referenceEdge & 0x000000ff);
	}
Features.prototype.get = function () {
		return _incidentEdge;
	}
Features.prototype.set = function (value) {
		_incidentEdge = value;
		_m_id._key = (_m_id._key & 0xffff00ff) | ((_incidentEdge << 8) & 0x0000ff00);
	}
Features.prototype.get = function () {
		return _incidentVertex;
	}
Features.prototype.set = function (value) {
		_incidentVertex = value;
		_m_id._key = (_m_id._key & 0xff00ffff) | ((_incidentVertex << 16) & 0x00ff0000);
	}
Features.prototype.get = function () {
		return _flip;
	}
Features.prototype.set = function (value) {
		_flip = value;
		_m_id._key = (_m_id._key & 0x00ffffff) | ((_flip << 24) & 0xff000000);
	}