var b2CircleContact = function() {
b2Contact.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2CircleContact.prototype, b2Contact.prototype)
b2CircleContact.prototype._super = function(){ b2Contact.prototype.__constructor.apply(this, arguments) }
b2CircleContact.prototype.__constructor = function(){}
b2CircleContact.prototype.__varz = function(){
}
// static methods
b2CircleContact.Create = function (allocator) {
		return new b2CircleContact();
	}
b2CircleContact.Destroy = function (contact, allocator) {
		
	}
// static attributes
// methods
b2CircleContact.prototype.Reset = function (fixtureA, fixtureB) {
		this._super.Reset(fixtureA, fixtureB);
		
		
	}
// attributes