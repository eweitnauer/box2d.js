var b2World = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2World.prototype.__constructor = function (gravity, doSleep) {
		
		this.m_destructionListener = null;
		this.m_debugDraw = null;
		
		m_bodyList = null;
		m_contactList = null;
		this.m_jointList = null;
		this.m_controllerList = null;
		
		this.m_bodyCount = 0;
		m_contactCount = 0;
		this.m_jointCount = 0;
		this.m_controllerCount = 0;
		
		m_warmStarting = true;
		m_continuousPhysics = true;
		
		this.m_allowSleep = doSleep;
		this.m_gravity = gravity;
		
		this.m_inv_dt0 = 0.0;
		
		m_contactManager.m_world = this;
		
		var bd = new b2BodyDef();
		m_groundBody = this.CreateBody(bd);
	}
b2World.prototype.__varz = function(){
this.s_stack =  new Vector();
this.m_contactSolver =  new b2ContactSolver();
this.m_island =  new b2Island();
}
// static attributes
b2World.s_timestep2 =  new b2TimeStep();
b2World.s_backupA =  new b2Sweep();
b2World.s_backupB =  new b2Sweep();
b2World.s_timestep =  new b2TimeStep();
b2World.s_queue =  new Vector();
b2World.e_newFixture =  0x0001;
b2World.e_locked =  0x0002;
b2World.s_xf =  new b2Transform();
b2World.s_jointColor =  new b2Color(0.5, 0.8, 0.8);
b2World. m_warmStarting =  null;
b2World. m_continuousPhysics =  null;
// static methods
// attributes
b2World.prototype.s_stack =  new Vector();
b2World.prototype.m_contactSolver =  new b2ContactSolver();
b2World.prototype.m_island =  new b2Island();
b2World.prototype.m_jointList =  null;
b2World.prototype.m_bodyCount =  0;
b2World.prototype.m_jointCount =  0;
b2World.prototype.m_controllerList =  null;
b2World.prototype.m_controllerCount =  0;
b2World.prototype.m_gravity =  null;
b2World.prototype.m_allowSleep =  null;
b2World.prototype.m_destructionListener =  null;
b2World.prototype.m_debugDraw =  null;
b2World.prototype.m_inv_dt0 =  null;
// methods
b2World.prototype.SetDestructionListener = function (listener) {
		this.m_destructionListener = listener;
	}
b2World.prototype.SetContactFilter = function (filter) {
		m_contactManager.m_contactFilter = filter;
	}
b2World.prototype.SetContactListener = function (listener) {
		m_contactManager.m_contactListener = listener;
	}
b2World.prototype.SetDebugDraw = function (debugDraw) {
		this.m_debugDraw = debugDraw;
	}
b2World.prototype.SetBroadPhase = function (broadPhase) {
		var oldBroadPhase = m_contactManager.m_broadPhase;
		m_contactManager.m_broadPhase = broadPhase;
		for (var b = m_bodyList; b; b = b.m_next)
		{
			for (var f = b.m_fixtureList; f; f = f.m_next)
			{
				f.m_proxy = broadPhase.CreateProxy(oldBroadPhase.GetFatAABB(f.m_proxy), f);
			}
		}
	}
b2World.prototype.Validate = function () {
		m_contactManager.m_broadPhase.Validate();
	}
b2World.prototype.GetProxyCount = function () {
		return m_contactManager.m_broadPhase.GetProxyCount();
	}
b2World.prototype.CreateBody = function (def) {
		
		
		if (this.IsLocked() == true)
		{
			return null;
		}
		
		
		var b = new b2Body(def, this);
		
		
		b.m_prev = null;
		b.m_next = m_bodyList;
		if (m_bodyList)
		{
			m_bodyList.m_prev = b;
		}
		m_bodyList = b;
		++this.m_bodyCount;
		
		return b;
		
	}
b2World.prototype.DestroyBody = function (b) {
		
		
		
		if (this.IsLocked() == true)
		{
			return;
		}
		
		
		var jn = b.m_jointList;
		while (jn)
		{
			var jn0 = jn;
			jn = jn.next;
			
			if (this.m_destructionListener)
			{
				this.m_destructionListener.SayGoodbyeJoint(jn0.joint);
			}
			
			this.DestroyJoint(jn0.joint);
		}
		
		
		var coe = b.m_controllerList;
		while (coe)
		{
			var coe0 = coe;
			coe = coe.nextController;
			coe0.controller.RemoveBody(b);
		}
		
		
		var ce = b.m_contactList;
		while (ce)
		{
			var ce0 = ce;
			ce = ce.next;
			m_contactManager.Destroy(ce0.contact);
		}
		b.m_contactList = null;
		
		
		
		var f = b.m_fixtureList;
		while (f)
		{
			var f0 = f;
			f = f.m_next;
			
			if (this.m_destructionListener)
			{
				this.m_destructionListener.SayGoodbyeFixture(f0);
			}
			
			f0.DestroyProxy(m_contactManager.m_broadPhase);
			f0.Destroy();
			
			
			
		}
		b.m_fixtureList = null;
		b.m_fixtureCount = 0;
		
		
		if (b.m_prev)
		{
			b.m_prev.m_next = b.m_next;
		}
		
		if (b.m_next)
		{
			b.m_next.m_prev = b.m_prev;
		}
		
		if (b == m_bodyList)
		{
			m_bodyList = b.m_next;
		}
		
		--this.m_bodyCount;
		
		
		
	}
b2World.prototype.CreateJoint = function (def) {
		
		
		
		var j = b2Joint.Create(def, null);
		
		
		j.m_prev = null;
		j.m_next = this.m_jointList;
		if (this.m_jointList)
		{
			this.m_jointList.m_prev = j;
		}
		this.m_jointList = j;
		++this.m_jointCount;
		
		
		j.m_edgeA.joint = j;
		j.m_edgeA.other = j.m_bodyB;
		j.m_edgeA.prev = null;
		j.m_edgeA.next = j.m_bodyA.m_jointList;
		if (j.m_bodyA.m_jointList) j.m_bodyA.m_jointList.prev = j.m_edgeA;
		j.m_bodyA.m_jointList = j.m_edgeA;
		
		j.m_edgeB.joint = j;
		j.m_edgeB.other = j.m_bodyA;
		j.m_edgeB.prev = null;
		j.m_edgeB.next = j.m_bodyB.m_jointList;
		if (j.m_bodyB.m_jointList) j.m_bodyB.m_jointList.prev = j.m_edgeB;
		j.m_bodyB.m_jointList = j.m_edgeB;
		
		var bodyA = def.bodyA;
		var bodyB = def.bodyB;
		
		
		if (def.collideConnected == false )
		{
			var edge = bodyB.GetContactList();
			while (edge)
			{
				if (edge.other == bodyA)
				{
					
					
					edge.contact.FlagForFiltering();
				}

				edge = edge.next;
			}
		}
		
		
		
		return j;
		
	}
b2World.prototype.DestroyJoint = function (j) {
		
		
		
		var collideConnected = j.m_collideConnected;
		
		
		if (j.m_prev)
		{
			j.m_prev.m_next = j.m_next;
		}
		
		if (j.m_next)
		{
			j.m_next.m_prev = j.m_prev;
		}
		
		if (j == this.m_jointList)
		{
			this.m_jointList = j.m_next;
		}
		
		
		var bodyA = j.m_bodyA;
		var bodyB = j.m_bodyB;
		
		
		bodyA.SetAwake(true);
		bodyB.SetAwake(true);
		
		
		if (j.m_edgeA.prev)
		{
			j.m_edgeA.prev.next = j.m_edgeA.next;
		}
		
		if (j.m_edgeA.next)
		{
			j.m_edgeA.next.prev = j.m_edgeA.prev;
		}
		
		if (j.m_edgeA == bodyA.m_jointList)
		{
			bodyA.m_jointList = j.m_edgeA.next;
		}
		
		j.m_edgeA.prev = null;
		j.m_edgeA.next = null;
		
		
		if (j.m_edgeB.prev)
		{
			j.m_edgeB.prev.next = j.m_edgeB.next;
		}
		
		if (j.m_edgeB.next)
		{
			j.m_edgeB.next.prev = j.m_edgeB.prev;
		}
		
		if (j.m_edgeB == bodyB.m_jointList)
		{
			bodyB.m_jointList = j.m_edgeB.next;
		}
		
		j.m_edgeB.prev = null;
		j.m_edgeB.next = null;
		
		b2Joint.Destroy(j, null);
		
		
		--this.m_jointCount;
		
		
		if (collideConnected == false)
		{
			var edge = bodyB.GetContactList();
			while (edge)
			{
				if (edge.other == bodyA)
				{
					
					
					edge.contact.FlagForFiltering();
				}

				edge = edge.next;
			}
		}
		
	}
b2World.prototype.AddController = function (c) {
		c.m_next = this.m_controllerList;
		c.m_prev = null;
		this.m_controllerList = c;
		
		c.m_world = this;
		
		this.m_controllerCount++;
		
		return c;
	}
b2World.prototype.RemoveController = function (c) {
		
		if (c.m_prev)
			c.m_prev.m_next = c.m_next;
		if (c.m_next)
			c.m_next.m_prev = c.m_prev;
		if (this.m_controllerList == c)
			this.m_controllerList = c.m_next;
			
		this.m_controllerCount--;
	}
b2World.prototype.CreateController = function (controller) {
		if (controller.m_world != this)
			throw new Error("Controller can only be a member of one world");
		
		controller.m_next = this.m_controllerList;
		controller.m_prev = null;
		if (this.m_controllerList)
			this.m_controllerList.m_prev = controller;
		this.m_controllerList = controller;
		++this.m_controllerCount;
		
		controller.m_world = this;
		
		return controller;
	}
b2World.prototype.DestroyController = function (controller) {
		
		controller.Clear();
		if (controller.m_next)
			controller.m_next.m_prev = controller.m_prev;
		if (controller.m_prev)
			controller.m_prev.m_next = controller.m_next;
		if (controller == this.m_controllerList)
			this.m_controllerList = controller.m_next;
		--this.m_controllerCount;
	}
b2World.prototype.SetWarmStarting = function (flag) {b2World. m_warmStarting = flag; }
b2World.prototype.SetContinuousPhysics = function (flag) {b2World. m_continuousPhysics = flag; }
b2World.prototype.GetBodyCount = function () {
		return this.m_bodyCount;
	}
b2World.prototype.GetJointCount = function () {
		return this.m_jointCount;
	}
b2World.prototype.GetContactCount = function () {
		return m_contactCount;
	}
b2World.prototype.SetGravity = function (gravity) {
		this.m_gravity = gravity;
	}
b2World.prototype.GetGravity = function () {
		return this.m_gravity;
	}
b2World.prototype.GetGroundBody = function () {
		return m_groundBody;
	}
b2World.prototype.Step = function (dt, velocityIterations, positionIterations) {
		if (m_flags & b2World.e_newFixture)
		{
			m_contactManager.FindNewContacts();
			m_flags &= ~b2World.e_newFixture;
		}
		
		m_flags |= b2World.e_locked;
		
		var step = b2World.s_timestep2;
		step.dt = dt;
		step.velocityIterations = velocityIterations;
		step.positionIterations = positionIterations;
		if (dt > 0.0)
		{
			step.inv_dt = 1.0 / dt;
		}
		else
		{
			step.inv_dt = 0.0;
		}
		
		step.dtRatio = this.m_inv_dt0 * dt;
		
		step.warmStarting =b2World. m_warmStarting;
		
		
		m_contactManager.Collide();
		
		
		if (step.dt > 0.0)
		{
			Solve(step);
		}
		
		
		if (m_continuousPhysics && step.dt > 0.0)
		{
			SolveTOI(step);
		}
		
		if (step.dt > 0.0)
		{
			this.m_inv_dt0 = step.inv_dt;
		}
		m_flags &= ~b2World.e_locked;
	}
b2World.prototype.ClearForces = function () {
		for (var body = m_bodyList; body; body = body.m_next)
		{
			body.m_force.SetZero();
			body.m_torque = 0.0;
		}
	}
b2World.prototype.DrawDebugData = function () {
		
		if (this.m_debugDraw == null)
		{
			return;
		}
		
		this.m_debugDraw.m_sprite.graphics.clear();
		
		var flags = this.m_debugDraw.GetFlags();
		
		var i = 0;
		var b;
		var f;
		var s;
		var j;
		var bp;
		var invQ = new b2Vec2;
		var x1 = new b2Vec2;
		var x2 = new b2Vec2;
		var xf;
		var b1 = new b2AABB();
		var b2 = new b2AABB();
		var vs = [new b2Vec2(), new b2Vec2(), new b2Vec2(), new b2Vec2()];
		
		
		var color = new b2Color(0, 0, 0);
			
		if (flags & b2DebugDraw.e_shapeBit)
		{
			for (b = m_bodyList; b; b = b.m_next)
			{
				xf = b.m_xf;
				for (f = b.GetFixtureList(); f; f = f.m_next)
				{
					s = f.GetShape();
					if (b.IsActive() == false)
					{
						color.Set(0.5, 0.5, 0.3);
						DrawShape(s, xf, color);
					}
					else if (b.GetType() == b2Body.b2_staticBody)
					{
						color.Set(0.5, 0.9, 0.5);
						DrawShape(s, xf, color);
					}
					else if (b.GetType() == b2Body.b2_kinematicBody)
					{
						color.Set(0.5, 0.5, 0.9);
						DrawShape(s, xf, color);
					}
					else if (b.IsAwake() == false)
					{
						color.Set(0.6, 0.6, 0.6);
						DrawShape(s, xf, color);
					}
					else
					{
						color.Set(0.9, 0.7, 0.7);
						DrawShape(s, xf, color);
					}
				}
			}
		}
		
		if (flags & b2DebugDraw.e_jointBit)
		{
			for (j = this.m_jointList; j; j = j.m_next)
			{
				DrawJoint(j);
			}
		}
		
		if (flags & b2DebugDraw.e_controllerBit)
		{
			for (var c = this.m_controllerList; c; c = c.m_next)
			{
				c.Draw(this.m_debugDraw);
			}
		}
		
		if (flags & b2DebugDraw.e_pairBit)
		{
			color.Set(0.3, 0.9, 0.9);
			for (var contact = m_contactManager.m_contactList; contact; contact = contact.GetNext())
			{
				var fixtureA = contact.GetFixtureA();
				var fixtureB = contact.GetFixtureB();

				var cA = fixtureA.GetAABB().GetCenter();
				var cB = fixtureB.GetAABB().GetCenter();

				this.m_debugDraw.DrawSegment(cA, cB, color);
			}
		}
		
		if (flags & b2DebugDraw.e_aabbBit)
		{
			bp = m_contactManager.m_broadPhase;
			
			vs = [new b2Vec2(),new b2Vec2(),new b2Vec2(),new b2Vec2()];
			
			for (b= m_bodyList; b; b = b.GetNext())
			{
				if (b.IsActive() == false)
				{
					continue;
				}
				for (f = b.GetFixtureList(); f; f = f.GetNext())
				{
					var aabb = bp.GetFatAABB(f.m_proxy);
					vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
					vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
					vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
					vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);

					this.m_debugDraw.DrawPolygon(vs, 4, color);
				}
			}
		}
		
		if (flags & b2DebugDraw.e_centerOfMassBit)
		{
			for (b = m_bodyList; b; b = b.m_next)
			{
				xf = b2World.s_xf;
				xf.R = b.m_xf.R;
				xf.position = b.GetWorldCenter();
				this.m_debugDraw.DrawTransform(xf);
			}
		}
	}
b2World.prototype.QueryAABB = function (callback, aabb) {
		var broadPhase = m_contactManager.m_broadPhase;
		function WorldQueryWrapper(proxy):Boolean
		{
			return callback(broadPhase.GetUserData(proxy));
		}
		broadPhase.Query(WorldQueryWrapper, aabb);
	}
b2World.prototype.QueryShape = function (callback, shape, transform ) {
		if (transform == null)
		{
			transform = new b2Transform();
			transform.SetIdentity();
		}
		var broadPhase = m_contactManager.m_broadPhase;
		function WorldQueryWrapper(proxy):Boolean
		{
			var fixture = broadPhase.GetUserData(proxy)
			if(b2Shape.TestOverlap(shape, transform, fixture.GetShape(), fixture.GetBody().GetTransform()))
				return callback(fixture);
			return true;
		}
		var aabb = new b2AABB();
		shape.ComputeAABB(aabb, transform);
		broadPhase.Query(WorldQueryWrapper, aabb);
	}
b2World.prototype.QueryPoint = function (callback, p) {
		var broadPhase = m_contactManager.m_broadPhase;
		function WorldQueryWrapper(proxy):Boolean
		{
			var fixture = broadPhase.GetUserData(proxy)
			if(fixture.TestPoint(p))
				return callback(fixture);
			return true;
		}
		
		var aabb = new b2AABB();
		aabb.lowerBound.Set(p.x - b2Settings.b2_linearSlop, p.y - b2Settings.b2_linearSlop);
		aabb.upperBound.Set(p.x + b2Settings.b2_linearSlop, p.y + b2Settings.b2_linearSlop);
		broadPhase.Query(WorldQueryWrapper, aabb);
	}
b2World.prototype.RayCast = function (callback, point1, point2) {
		var broadPhase = m_contactManager.m_broadPhase;
		var output = new b2RayCastOutput;
		function RayCastWrapper(input, proxy):Number
		{
			var userData = broadPhase.GetUserData(proxy);
			var fixture = userData;
			var hit = fixture.RayCast(output, input);
			if (hit)
			{
				var fraction = output.fraction;
				var point = new b2Vec2(
					(1.0 - fraction) * point1.x + fraction * point2.x,
					(1.0 - fraction) * point1.y + fraction * point2.y);
				return callback(fixture, point, output.normal, fraction);
			}
			return input.maxFraction;
		}
		var input = new b2RayCastInput(point1, point2);
		broadPhase.RayCast(RayCastWrapper, input);
	}
b2World.prototype.RayCastOne = function (point1, point2) {
		var result;
		function RayCastOneWrapper(fixture, point, normal, fraction):Number
		{
			result = fixture;
			return fraction;
		}
		this.RayCast(RayCastOneWrapper, point1, point2);
		return result;
	}
b2World.prototype.RayCastAll = function (point1, point2) {
		var result = new Vector();
		function RayCastAllWrapper(fixture, point, normal, fraction):Number
		{
			result[result.length] = fixture;
			return 1;
		}
		this.RayCast(RayCastAllWrapper, point1, point2);
		return result;
	}
b2World.prototype.GetBodyList = function () {
		return m_bodyList;
	}
b2World.prototype.GetJointList = function () {
		return this.m_jointList;
	}
b2World.prototype.GetContactList = function () {
		return m_contactList;
	}
b2World.prototype.IsLocked = function () {
		return (m_flags & b2World.e_locked) > 0;
	}