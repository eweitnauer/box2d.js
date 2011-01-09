var b2Fixture = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Fixture.prototype.__constructor = function () {
		m_aabb = new b2AABB();
		m_userData = null;
		m_body = null;
		m_next = null;
		
		m_shape = null;
		m_density = 0.0;
		
		m_friction = 0.0;
		m_restitution = 0.0;
	}
b2Fixture.prototype.__varz = function(){
}
// static attributes
// static methods
// attributes
b2Fixture.prototype.m_massData =  null;
// methods
b2Fixture.prototype.GetType = function () {
		return m_shape.GetType();
	}
b2Fixture.prototype.GetShape = function () {
		return m_shape;
	}
b2Fixture.prototype.SetSensor = function (sensor) {
		if ( m_isSensor == sensor)
			return;
			
		m_isSensor = sensor;
		
		if (m_body == null)
			return;
			
		var edge = m_body.GetContactList();
		while (edge)
		{
			var contact = edge.contact;
			var fixtureA = contact.GetFixtureA();
			var fixtureB = contact.GetFixtureB();
			if (fixtureA == this || fixtureB == this)
				contact.SetSensor(fixtureA.IsSensor() || fixtureB.IsSensor());
			edge = edge.next;
		}
		
	}
b2Fixture.prototype.IsSensor = function () {
		return m_isSensor;
	}
b2Fixture.prototype.SetFilterData = function (filter) {
		m_filter = filter.Copy();
		
		if (m_body)
			return;
			
		var edge = m_body.GetContactList();
		while (edge)
		{
			var contact = edge.contact;
			var fixtureA = contact.GetFixtureA();
			var fixtureB = contact.GetFixtureB();
			if (fixtureA == this || fixtureB == this)
				contact.FlagForFiltering();
			edge = edge.next;
		}
	}
b2Fixture.prototype.GetFilterData = function () {
		return m_filter.Copy();
	}
b2Fixture.prototype.GetBody = function () {
		return m_body;
	}
b2Fixture.prototype.GetNext = function () {
		return m_next;
	}
b2Fixture.prototype.GetUserData = function () {
		return m_userData;
	}
b2Fixture.prototype.SetUserData = function (data) {
		m_userData = data;
	}
b2Fixture.prototype.TestPoint = function (p) {
		return m_shape.TestPoint(m_body.GetTransform(), p);
	}
b2Fixture.prototype.RayCast = function (output, input) {
		return m_shape.RayCast(output, input, m_body.GetTransform());
	}
b2Fixture.prototype.GetMassData = function (massData ) {
		if ( massData == null )
		{
			massData = new b2MassData();
		}
		m_shape.ComputeMass(massData, m_density);
		return massData;
	}
b2Fixture.prototype.SetDensity = function (density) {
		
		m_density = density;
	}
b2Fixture.prototype.GetDensity = function () {
		return m_density;
	}
b2Fixture.prototype.GetFriction = function () {
		return m_friction;
	}
b2Fixture.prototype.SetFriction = function (friction) {
		m_friction = friction;
	}
b2Fixture.prototype.GetRestitution = function () {
		return m_restitution;
	}
b2Fixture.prototype.SetRestitution = function (restitution) {
		m_restitution = restitution;
	}
b2Fixture.prototype.GetAABB = function () {
		return m_aabb;
	}