var b2EdgeAndCircleContact = function() {
b2Contact.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2EdgeAndCircleContact.prototype, b2Contact.prototype)
b2EdgeAndCircleContact.prototype._super = function(){ b2Contact.prototype.__constructor.apply(this, arguments) }
b2EdgeAndCircleContact.prototype.__constructor = function(){}
b2EdgeAndCircleContact.prototype.__varz = function(){
}
// static attributes
// static methods
b2EdgeAndCircleContact.Create = function (allocator) {
		return new b2EdgeAndCircleContact();
	}
b2EdgeAndCircleContact.Destroy = function (contact, allocator) {
		
	}
// attributes
// methods
b2EdgeAndCircleContact.prototype.b2CollideEdgeAndCircle = function (manifold,
	 edge, 
	 xf1,
	 circle, 
	 xf2) {
		
		
	}
b2EdgeAndCircleContact.prototype.Reset = function (fixtureA, fixtureB) {
		super.Reset(fixtureA, fixtureB);
		
		
	}