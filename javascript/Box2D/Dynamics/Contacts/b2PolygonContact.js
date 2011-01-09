var b2PolygonContact = function() {
b2Contact.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2PolygonContact.prototype, b2Contact.prototype)
b2PolygonContact.prototype._super = function(){ b2Contact.prototype.__constructor.apply(this, arguments) }
b2PolygonContact.prototype.__constructor = function(){}
b2PolygonContact.prototype.__varz = function(){
}
// static attributes
// static methods
b2PolygonContact.Create = function (allocator) {
		
		return new b2PolygonContact();
	}
b2PolygonContact.Destroy = function (contact, allocator) {
		
		
	}
// attributes
// methods
b2PolygonContact.prototype.Reset = function (fixtureA, fixtureB) {
		this._super.Reset(fixtureA, fixtureB);
		
		
	}