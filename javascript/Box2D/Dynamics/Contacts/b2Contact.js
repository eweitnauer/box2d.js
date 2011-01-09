var b2Contact = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Contact.prototype.__constructor = function () {
		
	}
b2Contact.prototype.__varz = function(){
}
// static attributes
b2Contact.s_input =  new b2TOIInput();
// static methods
// attributes
// methods
b2Contact.prototype.GetManifold = function () {
		return m_manifold;
	}
b2Contact.prototype.GetWorldManifold = function (worldManifold) {
		var bodyA = m_fixtureA.GetBody();
		var bodyB = m_fixtureB.GetBody();
		var shapeA = m_fixtureA.GetShape();
		var shapeB = m_fixtureB.GetShape();
		
		worldManifold.Initialize(m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
	}
b2Contact.prototype.IsTouching = function () {
		return (m_flags & e_touchingFlag) == e_touchingFlag; 
	}
b2Contact.prototype.IsContinuous = function () {
		return (m_flags & e_continuousFlag) == e_continuousFlag; 
	}
b2Contact.prototype.SetSensor = function (sensor) {
		if (sensor)
		{
			m_flags |= e_sensorFlag;
		}
		else
		{
			m_flags &= ~e_sensorFlag;
		}
	}
b2Contact.prototype.IsSensor = function () {
		return (m_flags & e_sensorFlag) == e_sensorFlag;
	}
b2Contact.prototype.SetEnabled = function (flag) {
		if (flag)
		{
			m_flags |= e_enabledFlag;
		}
		else
		{
			m_flags &= ~e_enabledFlag;
		}
	}
b2Contact.prototype.IsEnabled = function () {
		return (m_flags & e_enabledFlag) == e_enabledFlag;
	}
b2Contact.prototype.GetNext = function () {
		return m_next;
	}
b2Contact.prototype.GetFixtureA = function () {
		return m_fixtureA;
	}
b2Contact.prototype.GetFixtureB = function () {
		return m_fixtureB;
	}
b2Contact.prototype.FlagForFiltering = function () {
		m_flags |= e_filterFlag;
	}