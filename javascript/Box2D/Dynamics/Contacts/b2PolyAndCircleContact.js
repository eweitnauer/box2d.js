var b2PolyAndCircleContact = function() {
b2Contact.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2PolyAndCircleContact.prototype, b2Contact.prototype)
b2PolyAndCircleContact.prototype._super = function(){ b2Contact.prototype.__constructor.apply(this, arguments) }
b2PolyAndCircleContact.prototype.__constructor = function(){}
b2PolyAndCircleContact.prototype.__varz = function(){
}
// static attributes
// static methods
b2PolyAndCircleContact.Create = function (allocator) {
		return new b2PolyAndCircleContact();
	}
b2PolyAndCircleContact.Destroy = function (contact, allocator) {
	}
// attributes
// methods
b2PolyAndCircleContact.prototype.Reset = function (fixtureA, fixtureB) {
		super.Reset(fixtureA, fixtureB);
		b2Settings.b2Assert(fixtureA.GetType() == b2Shape.e_polygonShape);
		b2Settings.b2Assert(fixtureB.GetType() == b2Shape.e_circleShape);
	}