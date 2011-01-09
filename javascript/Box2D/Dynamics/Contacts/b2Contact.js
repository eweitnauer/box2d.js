var b2Contact = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Contact.prototype.__constructor = function () {
		
	}
b2Contact.prototype.__varz = function(){
this.e_sensorFlag =  0x0001;
this.e_continuousFlag =  0x0002;
this.e_islandFlag =  0x0004;
this.e_toiFlag =  0x0008;
this.e_touchingFlag =  0x0010;
this.e_enabledFlag =  0x0020;
this.e_filterFlag =  0x0040;
this.m_nodeA =  new b2ContactEdge();
this.m_nodeB =  new b2ContactEdge();
this.m_manifold =  new b2Manifold();
this.m_oldManifold =  new b2Manifold();
}
// static attributes
b2Contact.s_input =  new b2TOIInput();
// static methods
// attributes
b2Contact.prototype.e_sensorFlag =  0x0001;
b2Contact.prototype.e_continuousFlag =  0x0002;
b2Contact.prototype.e_islandFlag =  0x0004;
b2Contact.prototype.e_toiFlag =  0x0008;
b2Contact.prototype.e_touchingFlag =  0x0010;
b2Contact.prototype.e_enabledFlag =  0x0020;
b2Contact.prototype.e_filterFlag =  0x0040;
b2Contact.prototype.m_flags =  0;
b2Contact.prototype.m_prev =  null;
b2Contact.prototype.m_next =  null;
b2Contact.prototype.m_nodeA =  new b2ContactEdge();
b2Contact.prototype.m_nodeB =  new b2ContactEdge();
b2Contact.prototype.m_fixtureA =  null;
b2Contact.prototype.m_fixtureB =  null;
b2Contact.prototype.m_manifold =  new b2Manifold();
b2Contact.prototype.m_oldManifold =  new b2Manifold();
b2Contact.prototype.m_toi =  null;
// methods
b2Contact.prototype.GetManifold = function () {
		return this.m_manifold;
	}
b2Contact.prototype.GetWorldManifold = function (worldManifold) {
		var bodyA = this.m_fixtureA.GetBody();
		var bodyB = this.m_fixtureB.GetBody();
		var shapeA = this.m_fixtureA.GetShape();
		var shapeB = this.m_fixtureB.GetShape();
		
		worldManifold.Initialize(this.m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
	}
b2Contact.prototype.IsTouching = function () {
		return (this.m_flags & this.e_touchingFlag) == this.e_touchingFlag; 
	}
b2Contact.prototype.IsContinuous = function () {
		return (this.m_flags & this.e_continuousFlag) == this.e_continuousFlag; 
	}
b2Contact.prototype.SetSensor = function (sensor) {
		if (sensor)
		{
			this.m_flags |= this.e_sensorFlag;
		}
		else
		{
			this.m_flags &= ~this.e_sensorFlag;
		}
	}
b2Contact.prototype.IsSensor = function () {
		return (this.m_flags & this.e_sensorFlag) == this.e_sensorFlag;
	}
b2Contact.prototype.SetEnabled = function (flag) {
		if (flag)
		{
			this.m_flags |= this.e_enabledFlag;
		}
		else
		{
			this.m_flags &= ~this.e_enabledFlag;
		}
	}
b2Contact.prototype.IsEnabled = function () {
		return (this.m_flags & this.e_enabledFlag) == this.e_enabledFlag;
	}
b2Contact.prototype.GetNext = function () {
		return this.m_next;
	}
b2Contact.prototype.GetFixtureA = function () {
		return this.m_fixtureA;
	}
b2Contact.prototype.GetFixtureB = function () {
		return this.m_fixtureB;
	}
b2Contact.prototype.FlagForFiltering = function () {
		this.m_flags |= this.e_filterFlag;
	}