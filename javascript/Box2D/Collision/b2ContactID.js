var b2ContactID = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2ContactID.prototype.__constructor = function () {
		this.features._m_id = this;
		
	}
b2ContactID.prototype.__varz = function(){
this.features =  new Features();
}
// static attributes
// static methods
// attributes
b2ContactID.prototype.features =  new Features();
// methods
b2ContactID.prototype.Set = function (id) {
		key = id._key;
	}
b2ContactID.prototype.Copy = function () {
		var id = new b2ContactID();
		id.key = key;
		return id;
	}
b2ContactID.prototype.get = function () {
		return _key;
	}
b2ContactID.prototype.set = function (value) {
		_key = value;
		this.features._referenceEdge = _key & 0x000000ff;
		this.features._incidentEdge = ((_key & 0x0000ff00) >> 8) & 0x000000ff;
		this.features._incidentVertex = ((_key & 0x00ff0000) >> 16) & 0x000000ff;
		this.features._flip = ((_key & 0xff000000) >> 24) & 0x000000ff;
	}