var b2Body = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Body.prototype.__constructor = function (bd, world) {
		
		
		
 		
 		
 		
 		
 		
 		
		
		m_flags = 0;
		
		if (bd.bullet )
		{
			m_flags |= e_bulletFlag;
		}
		if (bd.fixedRotation)
		{
			m_flags |= e_fixedRotationFlag;
		}
		if (bd.allowSleep)
		{
			m_flags |= e_allowSleepFlag;
		}
		if (bd.awake)
		{
			m_flags |= e_awakeFlag;
		}
		if (bd.active)
		{
			m_flags |= e_activeFlag;
		}
		
		m_world = world;
		
		m_xf.position.SetV(bd.position);
		m_xf.R.Set(bd.angle);
		
		m_sweep.localCenter.SetZero();
		m_sweep.t0 = 1.0;
		m_sweep.a0 = m_sweep.a = bd.angle;
		
		
		
		var tMat = m_xf.R;
		var tVec = m_sweep.localCenter;
		
		m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		
		m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		m_sweep.c.x += m_xf.position.x;
		m_sweep.c.y += m_xf.position.y;
		
		m_sweep.c0.SetV(m_sweep.c);
		
		m_jointList = null;
		m_controllerList = null;
		m_contactList = null;
		m_controllerCount = 0;
		m_prev = null;
		m_next = null;
		
		m_linearVelocity.SetV(bd.linearVelocity);
		m_angularVelocity = bd.angularVelocity;
		
		m_linearDamping = bd.linearDamping;
		m_angularDamping = bd.angularDamping;
		
		m_force.Set(0.0, 0.0);
		m_torque = 0.0;
		
		m_sleepTime = 0.0;
		
		m_type = bd.type;
		
		if (m_type == b2Body.b2_dynamicBody)
		{
			m_mass = 1.0;
			m_invMass = 1.0;
		}
		else
		{
			m_mass = 0.0;
			m_invMass = 0.0;
		}
		
		m_I = 0.0;
		m_invI = 0.0;
		
		m_inertiaScale = bd.inertiaScale;
		
		this.m_userData = bd.userData;
		
		m_fixtureList = null;
		m_fixtureCount = 0;
	}
b2Body.prototype.__varz = function(){
}
// static attributes
b2Body.b2_staticBody =  0;
b2Body.b2_kinematicBody =  1;
b2Body.b2_dynamicBody =  2;
b2Body.s_xf1 =  new b2Transform();
// static methods
// attributes
b2Body.prototype.m_userData =  null;
// methods
b2Body.prototype.connectEdges = function (s1, s2, angle1) {
		var angle2 = Math.atan2(s2.GetDirectionVector().y, s2.GetDirectionVector().x);
		var coreOffset = Math.tan((angle2 - angle1) * 0.5);
		var core = b2Math.MulFV(coreOffset, s2.GetDirectionVector());
		core = b2Math.SubtractVV(core, s2.GetNormalVector());
		core = b2Math.MulFV(b2Settings.b2_toiSlop, core);
		core = b2Math.AddVV(core, s2.GetVertex1());
		var cornerDir = b2Math.AddVV(s1.GetDirectionVector(), s2.GetDirectionVector());
		cornerDir.Normalize();
		var convex = b2Math.Dot(s1.GetDirectionVector(), s2.GetNormalVector()) > 0.0;
		s1.SetNextEdge(s2, core, cornerDir, convex);
		s2.SetPrevEdge(s1, core, cornerDir, convex);
		return angle2;
	}
b2Body.prototype.CreateFixture = function (def) {
		
		if (m_world.IsLocked() == true)
		{
			return null;
		}
		
		
		
		
		
		
		var fixture = new b2Fixture();
		fixture.Create(this, m_xf, def);
		
		if ( m_flags & e_activeFlag )
		{
			var broadPhase = m_world.m_contactManager.m_broadPhase;
			fixture.CreateProxy(broadPhase, m_xf);
		}
		
		fixture.m_next = m_fixtureList;
		m_fixtureList = fixture;
		++m_fixtureCount;
		
		fixture.m_body = this;
		
		
		if (fixture.m_density > 0.0)
		{
			this.ResetMassData();
		}
		
		
		
		m_world.m_flags |= b2World.e_newFixture;
		
		return fixture;
	}
b2Body.prototype.CreateFixture2 = function (shape, density) {
		var def = new b2FixtureDef();
		def.shape = shape;
		def.density = density;
		
		return this.CreateFixture(def);
	}
b2Body.prototype.DestroyFixture = function (fixture) {
		
		if (m_world.IsLocked() == true)
		{
			return;
		}
		
		
		
		var node = m_fixtureList;
		var ppF = null; 
		var found = false;
		while (node != null)
		{
			if (node == fixture)
			{
				if (ppF)
					ppF.m_next = fixture.m_next;
				else
					m_fixtureList = fixture.m_next;
				
				found = true;
				break;
			}
			
			ppF = node;
			node = node.m_next;
		}
		
		
		
		
		
		var edge = m_contactList;
		while (edge)
		{
			var c = edge.contact;
			edge = edge.next;
			
			var fixtureA = c.GetFixtureA();
			var fixtureB = c.GetFixtureB();
			if (fixture == fixtureA || fixture == fixtureB)
			{
				
				
				m_world.m_contactManager.Destroy(c);
			}
		}
		
		if ( m_flags & e_activeFlag )
		{
			var broadPhase = m_world.m_contactManager.m_broadPhase;
			fixture.DestroyProxy(broadPhase);
		}
		else
		{
			
		}
		
		fixture.Destroy();
		fixture.m_body = null;
		fixture.m_next = null;
		
		--m_fixtureCount;
		
		
		this.ResetMassData();
	}
b2Body.prototype.SetPositionAndAngle = function (position, angle) {
		
		var f;
		
		
		if (m_world.IsLocked() == true)
		{
			return;
		}
		
		m_xf.R.Set(angle);
		m_xf.position.SetV(position);
		
		
		
		var tMat = m_xf.R;
		var tVec = m_sweep.localCenter;
		
		m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		
		m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		m_sweep.c.x += m_xf.position.x;
		m_sweep.c.y += m_xf.position.y;
		
		m_sweep.c0.SetV(m_sweep.c);
		
		m_sweep.a0 = m_sweep.a = angle;
		
		var broadPhase = m_world.m_contactManager.m_broadPhase;
		for (f = m_fixtureList; f; f = f.m_next)
		{
			f.Synchronize(broadPhase, m_xf, m_xf);
		}
		m_world.m_contactManager.FindNewContacts();
	}
b2Body.prototype.SetTransform = function (xf) {
		this.SetPositionAndAngle(xf.position, xf.GetAngle());
	}
b2Body.prototype.GetTransform = function () {
		return m_xf;
	}
b2Body.prototype.GetPosition = function () {
		return m_xf.position;
	}
b2Body.prototype.SetPosition = function (position) {
		this.SetPositionAndAngle(position, this.GetAngle());
	}
b2Body.prototype.GetAngle = function () {
		return m_sweep.a;
	}
b2Body.prototype.SetAngle = function (angle) {
		this.SetPositionAndAngle(this.GetPosition(), angle);
	}
b2Body.prototype.GetWorldCenter = function () {
		return m_sweep.c;
	}
b2Body.prototype.GetLocalCenter = function () {
		return m_sweep.localCenter;
	}
b2Body.prototype.SetLinearVelocity = function (v) {
		if ( m_type == b2Body.b2_staticBody )
		{
			return;
		}
		m_linearVelocity.SetV(v);
	}
b2Body.prototype.GetLinearVelocity = function () {
		return m_linearVelocity;
	}
b2Body.prototype.SetAngularVelocity = function (omega) {
		if ( m_type == b2Body.b2_staticBody )
		{
			return;
		}
		m_angularVelocity = omega;
	}
b2Body.prototype.GetAngularVelocity = function () {
		return m_angularVelocity;
	}
b2Body.prototype.GetDefinition = function () {
		var bd = new b2BodyDef();
		bd.type = this.GetType();
		bd.allowSleep = (m_flags & e_allowSleepFlag) == e_allowSleepFlag;
		bd.angle = this.GetAngle();
		bd.angularDamping = m_angularDamping;
		bd.angularVelocity = m_angularVelocity;
		bd.fixedRotation = (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
		bd.bullet = (m_flags & e_bulletFlag) == e_bulletFlag;
		bd.awake = (m_flags & e_awakeFlag) == e_awakeFlag;
		bd.linearDamping = m_linearDamping;
		bd.linearVelocity.SetV(this.GetLinearVelocity());
		bd.position = this.GetPosition();
		bd.userData = this.GetUserData();
		return bd;
	}
b2Body.prototype.ApplyForce = function (force, point) {
		if (m_type != b2Body.b2_dynamicBody)
		{
			return;
		}
		
		if (this.IsAwake() == false)
		{
			this.SetAwake(true);
		}
		
		
		m_force.x += force.x;
		m_force.y += force.y;
		
		m_torque += ((point.x - m_sweep.c.x) * force.y - (point.y - m_sweep.c.y) * force.x);
	}
b2Body.prototype.ApplyTorque = function (torque) {
		if (m_type != b2Body.b2_dynamicBody)
		{
			return;
		}
		
		if (this.IsAwake() == false)
		{
			this.SetAwake(true);
		}
		m_torque += torque;
	}
b2Body.prototype.ApplyImpulse = function (impulse, point) {
		if (m_type != b2Body.b2_dynamicBody)
		{
			return;
		}
		
		if (this.IsAwake() == false)
		{
			this.SetAwake(true);
		}
		
		m_linearVelocity.x += m_invMass * impulse.x;
		m_linearVelocity.y += m_invMass * impulse.y;
		
		m_angularVelocity += m_invI * ((point.x - m_sweep.c.x) * impulse.y - (point.y - m_sweep.c.y) * impulse.x);
	}
b2Body.prototype.Split = function (callback) {
		var linearVelocity = this.GetLinearVelocity().Copy();
		var angularVelocity = this.GetAngularVelocity();
		var center = this.GetWorldCenter();
		var body1 = this;
		var body2 = m_world.CreateBody(this.GetDefinition());
		
		var prev;
		for (var f = body1.m_fixtureList; f; )
		{
			if (callback(f))
			{
				var next = f.m_next;
				
				if (prev)
				{
					prev.m_next = next;
				}else {
					body1.m_fixtureList = next;
				}
				body1.m_fixtureCount--;
				
				
				f.m_next = body2.m_fixtureList;
				body2.m_fixtureList = f;
				body2.m_fixtureCount++;
				
				f.m_body = body2;
				
				f = next;
			}else {
				prev = f;
				f = f.m_next
			}
		}
		
		body1.ResetMassData();
		body2.ResetMassData();
		
		
		var center1 = body1.GetWorldCenter();
		var center2 = body2.GetWorldCenter();
		
		var velocity1 = b2Math.AddVV(linearVelocity, 
			b2Math.CrossFV(angularVelocity,
				b2Math.SubtractVV(center1, center)));
				
		var velocity2 = b2Math.AddVV(linearVelocity, 
			b2Math.CrossFV(angularVelocity,
				b2Math.SubtractVV(center2, center)));
				
		body1.SetLinearVelocity(velocity1);
		body2.SetLinearVelocity(velocity2);
		body1.SetAngularVelocity(angularVelocity);
		body2.SetAngularVelocity(angularVelocity);
		
		body1.SynchronizeFixtures();
		body2.SynchronizeFixtures();
		
		return body2;
	}
b2Body.prototype.Merge = function (other) {
		var f;
		for (f = other.m_fixtureList; f; )
		{
			var next = f.m_next;
			
			
			other.m_fixtureCount--;
			
			
			f.m_next = m_fixtureList;
			m_fixtureList = f;
			m_fixtureCount++;
			
			f.m_body = body2;
			
			f = next;
		}
		body1.m_fixtureCount = 0;
		
		
		var body1 = this;
		var body2 = other;
		
		
		var center1 = body1.GetWorldCenter();
		var center2 = body2.GetWorldCenter();
		
		var velocity1 = body1.GetLinearVelocity().Copy();
		var velocity2 = body2.GetLinearVelocity().Copy();
		
		var angular1 = body1.GetAngularVelocity();
		var angular = body2.GetAngularVelocity();
		
		
		
		body1.ResetMassData();
		
		SynchronizeFixtures();
	}
b2Body.prototype.GetMass = function () {
		return m_mass;
	}
b2Body.prototype.GetInertia = function () {
		return m_I;
	}
b2Body.prototype.GetMassData = function (data) {
		data.mass = m_mass;
		data.I = m_I;
		data.center.SetV(m_sweep.localCenter);
	}
b2Body.prototype.SetMassData = function (massData) {
		b2Settings.b2Assert(m_world.IsLocked() == false);
		if (m_world.IsLocked() == true)
		{
			return;
		}
		
		if (m_type != b2Body.b2_dynamicBody)
		{
			return;
		}
		
		m_invMass = 0.0;
		m_I = 0.0;
		m_invI = 0.0;
		
		m_mass = massData.mass;
		
		
		if (m_mass <= 0.0)
		{
			m_mass = 1.0;
		}
		m_invMass = 1.0 / m_mass;
		
		if (massData.I > 0.0 && (m_flags & e_fixedRotationFlag) == 0)
		{
			
			m_I = massData.I - m_mass * (massData.center.x * massData.center.x + massData.center.y * massData.center.y);
			m_invI = 1.0 / m_I;
		}
		
		
		var oldCenter = m_sweep.c.Copy();
		m_sweep.localCenter.SetV(massData.center);
		m_sweep.c0.SetV(b2Math.MulX(m_xf, m_sweep.localCenter));
		m_sweep.c.SetV(m_sweep.c0);
		
		
		
		m_linearVelocity.x += m_angularVelocity * -(m_sweep.c.y - oldCenter.y);
		m_linearVelocity.y += m_angularVelocity * +(m_sweep.c.x - oldCenter.x);
		
	}
b2Body.prototype.ResetMassData = function () {
		
		m_mass = 0.0;
		m_invMass = 0.0;
		m_I = 0.0;
		m_invI = 0.0;
		m_sweep.localCenter.SetZero();
		
		
		if (m_type == b2Body.b2_staticBody || m_type == b2Body.b2_kinematicBody)
		{
			return;
		}
		
		
		
		var center = b2Vec2.Make(0, 0);
		for (var f = m_fixtureList; f; f = f.m_next)
		{
			if (f.m_density == 0.0)
			{
				continue;
			}
			
			var massData = f.GetMassData();
			m_mass += massData.mass;
			center.x += massData.center.x * massData.mass;
			center.y += massData.center.y * massData.mass;
			m_I += massData.I;
		}
		
		
		if (m_mass > 0.0)
		{
			m_invMass = 1.0 / m_mass;
			center.x *= m_invMass;
			center.y *= m_invMass;
		}
		else
		{
			
			m_mass = 1.0;
			m_invMass = 1.0;
		}
		
		if (m_I > 0.0 && (m_flags & e_fixedRotationFlag) == 0)
		{
			
			m_I -= m_mass * (center.x * center.x + center.y * center.y);
			m_I *= m_inertiaScale;
			b2Settings.b2Assert(m_I > 0);
			m_invI = 1.0 / m_I;
		}else {
			m_I = 0.0;
			m_invI = 0.0;
		}
		
		
		var oldCenter = m_sweep.c.Copy();
		m_sweep.localCenter.SetV(center);
		m_sweep.c0.SetV(b2Math.MulX(m_xf, m_sweep.localCenter));
		m_sweep.c.SetV(m_sweep.c0);
		
		
		
		m_linearVelocity.x += m_angularVelocity * -(m_sweep.c.y - oldCenter.y);
		m_linearVelocity.y += m_angularVelocity * +(m_sweep.c.x - oldCenter.x);
		
	}
b2Body.prototype.GetWorldPoint = function (localPoint) {
		
		var A = m_xf.R;
		var u = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, 
								 A.col1.y * localPoint.x + A.col2.y * localPoint.y);
		u.x += m_xf.position.x;
		u.y += m_xf.position.y;
		return u;
	}
b2Body.prototype.GetWorldVector = function (localVector) {
		return b2Math.MulMV(m_xf.R, localVector);
	}
b2Body.prototype.GetLocalPoint = function (worldPoint) {
		return b2Math.MulXT(m_xf, worldPoint);
	}
b2Body.prototype.GetLocalVector = function (worldVector) {
		return b2Math.MulTMV(m_xf.R, worldVector);
	}
b2Body.prototype.GetLinearVelocityFromWorldPoint = function (worldPoint) {
		
		return new b2Vec2(m_linearVelocity.x - m_angularVelocity * (worldPoint.y - m_sweep.c.y), 
		 m_linearVelocity.y + m_angularVelocity * (worldPoint.x - m_sweep.c.x));
	}
b2Body.prototype.GetLinearVelocityFromLocalPoint = function (localPoint) {
		
		var A = m_xf.R;
		var worldPoint = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, 
		 A.col1.y * localPoint.x + A.col2.y * localPoint.y);
		worldPoint.x += m_xf.position.x;
		worldPoint.y += m_xf.position.y;
		return new b2Vec2(m_linearVelocity.x - m_angularVelocity * (worldPoint.y - m_sweep.c.y), 
		 m_linearVelocity.y + m_angularVelocity * (worldPoint.x - m_sweep.c.x));
	}
b2Body.prototype.GetLinearDamping = function () {
		return m_linearDamping;
	}
b2Body.prototype.SetLinearDamping = function (linearDamping) {
		m_linearDamping = linearDamping;
	}
b2Body.prototype.GetAngularDamping = function () {
		return m_angularDamping;
	}
b2Body.prototype.SetAngularDamping = function (angularDamping) {
		m_angularDamping = angularDamping;
	}
b2Body.prototype.SetType = function ( type ) {
		if ( m_type == type )
		{
			return;
		}
		
		m_type = type;
		
		this.ResetMassData();
		
		if ( m_type == b2Body.b2_staticBody )
		{
			m_linearVelocity.SetZero();
			m_angularVelocity = 0.0;
		}
		
		this.SetAwake(true);
		
		m_force.SetZero();
		m_torque = 0.0;
		
		
		for (var ce = m_contactList; ce; ce = ce.next)
		{
			ce.contact.FlagForFiltering();
		} 
	}
b2Body.prototype.GetType = function () {
		return m_type;
	}
b2Body.prototype.SetBullet = function (flag) {
		if (flag)
		{
			m_flags |= e_bulletFlag;
		}
		else
		{
			m_flags &= ~e_bulletFlag;
		}
	}
b2Body.prototype.IsBullet = function () {
		return (m_flags & e_bulletFlag) == e_bulletFlag;
	}
b2Body.prototype.SetSleepingAllowed = function (flag) {
		if (flag)
		{
			m_flags |= e_allowSleepFlag;
		}
		else
		{
			m_flags &= ~e_allowSleepFlag;
			this.SetAwake(true);
		}
	}
b2Body.prototype.SetAwake = function (flag) {
		if (flag)
		{
			m_flags |= e_awakeFlag;
			m_sleepTime = 0.0;
		}
		else
		{
			m_flags &= ~e_awakeFlag;
			m_sleepTime = 0.0;
			m_linearVelocity.SetZero();
			m_angularVelocity = 0.0;
			m_force.SetZero();
			m_torque = 0.0;
		}
	}
b2Body.prototype.IsAwake = function () {
		return (m_flags & e_awakeFlag) == e_awakeFlag;
	}
b2Body.prototype.SetFixedRotation = function (fixed) {
		if(fixed)
		{
			m_flags |= e_fixedRotationFlag;
		}
		else
		{
			m_flags &= ~e_fixedRotationFlag;
		}
		
		this.ResetMassData();
	}
b2Body.prototype.IsFixedRotation = function () {
		return (m_flags & e_fixedRotationFlag)==e_fixedRotationFlag;
	}
b2Body.prototype.SetActive = function ( flag ) {
		if (flag == this.IsActive())
		{
			return;
		}
		
		var broadPhase;
		var f;
		if (flag)
		{
			m_flags |= e_activeFlag;

			
			broadPhase = m_world.m_contactManager.m_broadPhase;
			for ( f = m_fixtureList; f; f = f.m_next)
			{
				f.CreateProxy(broadPhase, m_xf);
			}
			
		}
		else
		{
			m_flags &= ~e_activeFlag;

			
			broadPhase = m_world.m_contactManager.m_broadPhase;
			for ( f = m_fixtureList; f; f = f.m_next)
			{
				f.DestroyProxy(broadPhase);
			}

			
			var ce = m_contactList;
			while (ce)
			{
				var ce0 = ce;
				ce = ce.next;
				m_world.m_contactManager.Destroy(ce0.contact);
			}
			m_contactList = null;
		}
	}
b2Body.prototype.IsActive = function () {
		return (m_flags & e_activeFlag) == e_activeFlag;
	}
b2Body.prototype.IsSleepingAllowed = function () {
		return(m_flags & e_allowSleepFlag) == e_allowSleepFlag;
	}
b2Body.prototype.GetFixtureList = function () {
		return m_fixtureList;
	}
b2Body.prototype.GetJointList = function () {
		return m_jointList;
	}
b2Body.prototype.GetControllerList = function () {
		return m_controllerList;
	}
b2Body.prototype.GetContactList = function () {
		return m_contactList;
	}
b2Body.prototype.GetNext = function () {
		return m_next;
	}
b2Body.prototype.GetUserData = function () {
		return this.m_userData;
	}
b2Body.prototype.SetUserData = function (data) {
		this.m_userData = data;
	}
b2Body.prototype.GetWorld = function () {
		return m_world;
	}