var b2Fixture = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Fixture.prototype.__constructor = function () {
		this.m_aabb = new b2AABB();
		this.m_userData = null;
		this.m_body = null;
		this.m_next = null;
		
		this.m_shape = null;
		this.m_density = 0.0;
		
		this.m_friction = 0.0;
		this.m_restitution = 0.0;
	}
b2Fixture.prototype.__varz = function(){
this.m_filter =  new b2FilterData();
}
// static methods
// static attributes
// methods
b2Fixture.prototype.GetType = function () {
		return this.m_shape.GetType();
	}
b2Fixture.prototype.GetShape = function () {
		return this.m_shape;
	}
b2Fixture.prototype.SetSensor = function (sensor) {
		if ( this.m_isSensor == sensor)
			return;
			
		this.m_isSensor = sensor;
		
		if (this.m_body == null)
			return;
			
		var edge = this.m_body.GetContactList();
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
		return this.m_isSensor;
	}
b2Fixture.prototype.SetFilterData = function (filter) {
		this.m_filter = filter.Copy();
		
		if (this.m_body)
			return;
			
		var edge = this.m_body.GetContactList();
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
		return this.m_filter.Copy();
	}
b2Fixture.prototype.GetBody = function () {
		return this.m_body;
	}
b2Fixture.prototype.GetNext = function () {
		return this.m_next;
	}
b2Fixture.prototype.GetUserData = function () {
		return this.m_userData;
	}
b2Fixture.prototype.SetUserData = function (data) {
		this.m_userData = data;
	}
b2Fixture.prototype.TestPoint = function (p) {
		return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
	}
b2Fixture.prototype.RayCast = function (output, input) {
		return this.m_shape.RayCast(output, input, this.m_body.GetTransform());
	}
b2Fixture.prototype.GetMassData = function (massData ) {
		if ( massData == null )
		{
			massData = new b2MassData();
		}
		this.m_shape.ComputeMass(massData, this.m_density);
		return massData;
	}
b2Fixture.prototype.SetDensity = function (density) {
		
		this.m_density = density;
	}
b2Fixture.prototype.GetDensity = function () {
		return this.m_density;
	}
b2Fixture.prototype.GetFriction = function () {
		return this.m_friction;
	}
b2Fixture.prototype.SetFriction = function (friction) {
		this.m_friction = friction;
	}
b2Fixture.prototype.GetRestitution = function () {
		return this.m_restitution;
	}
b2Fixture.prototype.SetRestitution = function (restitution) {
		this.m_restitution = restitution;
	}
b2Fixture.prototype.GetAABB = function () {
		return this.m_aabb;
	}
// attributes
b2Fixture.prototype.m_massData =  null;
b2Fixture.prototype.m_aabb =  null;
b2Fixture.prototype.m_density =  null;
b2Fixture.prototype.m_next =  null;
b2Fixture.prototype.m_body =  null;
b2Fixture.prototype.m_shape =  null;
b2Fixture.prototype.m_friction =  null;
b2Fixture.prototype.m_restitution =  null;
b2Fixture.prototype.m_proxy =  null;
b2Fixture.prototype.m_filter =  new b2FilterData();
b2Fixture.prototype.m_isSensor =  null;
b2Fixture.prototype.m_userData =  null;