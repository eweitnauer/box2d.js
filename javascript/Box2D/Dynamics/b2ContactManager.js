var b2ContactManager = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2ContactManager.prototype.__constructor = function () {
		m_world = null;
		m_contactCount = 0;
		m_contactFilter = b2ContactFilter.b2_defaultFilter;
		m_contactListener = b2ContactListener.b2_defaultListener;
		m_contactFactory = new b2ContactFactory(m_allocator);
		m_broadPhase = new b2DynamicTreeBroadPhase();
	}
b2ContactManager.prototype.__varz = function(){
}
// static attributes
b2ContactManager.s_evalCP =  new b2ContactPoint();
// static methods
// attributes
// methods
b2ContactManager.prototype.AddPair = function (proxyUserDataA, proxyUserDataB) {
		var fixtureA = proxyUserDataA;
		var fixtureB = proxyUserDataB;
		
		var bodyA = fixtureA.GetBody();
		var bodyB = fixtureB.GetBody();
		
		
		if (bodyA == bodyB)
			return;
		
		
		var edge = bodyB.GetContactList();
		while (edge)
		{
			if (edge.other == bodyA)
			{
				var fA = edge.contact.GetFixtureA();
				var fB = edge.contact.GetFixtureB();
				if (fA == fixtureA && fB == fixtureB)
					return;
				if (fA == fixtureB && fB == fixtureA)
					return;
			}
			edge = edge.next;
		}
		
		
		if (bodyB.ShouldCollide(bodyA) == false)
		{
			return;
		}
		
		
		if (m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
		{
			return;
		}
		
		
		var c = m_contactFactory.Create(fixtureA, fixtureB);
		
		
		fixtureA = c.GetFixtureA();
		fixtureB = c.GetFixtureB();
		bodyA = fixtureA.m_body;
		bodyB = fixtureB.m_body;
		
		
		c.m_prev = null;
		c.m_next = m_world.m_contactList;
		if (m_world.m_contactList != null)
		{
			m_world.m_contactList.m_prev = c;
		}
		m_world.m_contactList = c;
		
		
		
		
		
		c.m_nodeA.contact = c;
		c.m_nodeA.other = bodyB;
		
		c.m_nodeA.prev = null;
		c.m_nodeA.next = bodyA.m_contactList;
		if (bodyA.m_contactList != null)
		{
			bodyA.m_contactList.prev = c.m_nodeA;
		}
		bodyA.m_contactList = c.m_nodeA;
		
		
		c.m_nodeB.contact = c;
		c.m_nodeB.other = bodyA;
		
		c.m_nodeB.prev = null;
		c.m_nodeB.next = bodyB.m_contactList;
		if (bodyB.m_contactList != null)
		{
			bodyB.m_contactList.prev = c.m_nodeB;
		}
		bodyB.m_contactList = c.m_nodeB;
		
		++m_world.m_contactCount;
		return;
		
	}
b2ContactManager.prototype.FindNewContacts = function () {
		m_broadPhase.UpdatePairs(this.AddPair);
	}
b2ContactManager.prototype.Destroy = function (c) {
		
		var fixtureA = c.GetFixtureA();
		var fixtureB = c.GetFixtureB();
		var bodyA = fixtureA.GetBody();
		var bodyB = fixtureB.GetBody();
		
		if (c.IsTouching())
		{
			m_contactListener.EndContact(c);
		}
		
		
		if (c.m_prev)
		{
			c.m_prev.m_next = c.m_next;
		}
		
		if (c.m_next)
		{
			c.m_next.m_prev = c.m_prev;
		}
		
		if (c == m_world.m_contactList)
		{
			m_world.m_contactList = c.m_next;
		}
		
		
		if (c.m_nodeA.prev)
		{
			c.m_nodeA.prev.next = c.m_nodeA.next;
		}
		
		if (c.m_nodeA.next)
		{
			c.m_nodeA.next.prev = c.m_nodeA.prev;
		}
		
		if (c.m_nodeA == bodyA.m_contactList)
		{
			bodyA.m_contactList = c.m_nodeA.next;
		}
		
		
		if (c.m_nodeB.prev)
		{
			c.m_nodeB.prev.next = c.m_nodeB.next;
		}
		
		if (c.m_nodeB.next)
		{
			c.m_nodeB.next.prev = c.m_nodeB.prev;
		}
		
		if (c.m_nodeB == bodyB.m_contactList)
		{
			bodyB.m_contactList = c.m_nodeB.next;
		}
		
		
		m_contactFactory.Destroy(c);
		--m_contactCount;
	}
b2ContactManager.prototype.Collide = function () {
		
		var c = m_world.m_contactList;
		while (c)
		{
			var fixtureA = c.GetFixtureA();
			var fixtureB = c.GetFixtureB();
			var bodyA = fixtureA.GetBody();
			var bodyB = fixtureB.GetBody();
			if (bodyA.IsAwake() == false && bodyB.IsAwake() == false)
			{
				c = c.GetNext();
				continue;
			}
			
			
			if (c.m_flags & b2Contact.e_filterFlag)
			{
				
				if (bodyB.ShouldCollide(bodyA) == false)
				{
					var cNuke = c;
					c = cNuke.GetNext();
					this.Destroy(cNuke);
					continue;
				}
				
				
				if (m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
				{
					cNuke = c;
					c = cNuke.GetNext();
					this.Destroy(cNuke);
					continue;
				}
				
				
				c.m_flags &= ~b2Contact.e_filterFlag;
			}
			
			var proxyA = fixtureA.m_proxy;
			var proxyB = fixtureB.m_proxy;
			
			var overlap = m_broadPhase.TestOverlap(proxyA, proxyB);
			
			
			if ( overlap == false)
			{
				cNuke = c;
				c = cNuke.GetNext();
				this.Destroy(cNuke);
				continue;
			}
			
			c.Update(m_contactListener);
			c = c.GetNext();
		}
	}