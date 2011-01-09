var b2ContactFactory = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2ContactFactory.prototype.__constructor = function(){}
b2ContactFactory.prototype.__varz = function(){
}
// static attributes
// static methods
// attributes
b2ContactFactory.prototype.m_registers =  null;
b2ContactFactory.prototype.m_allocator =  null;
// methods
b2ContactFactory.prototype.Create = function (fixtureA, fixtureB) {
		var type1 = fixtureA.GetType();
		var type2 = fixtureB.GetType();
		
		
		
		
		var reg = this.m_registers[type1][type2];
		
		var c;
		
		if (reg.pool)
		{
			
			c = reg.pool;
			reg.pool = c.m_next;
			reg.poolCount--;
			c.Reset(fixtureA, fixtureB);
			return c;
		}
		
		var createFcn = reg.createFcn;
		if (createFcn != null)
		{
			if (reg.primary)
			{
				c = createFcn(this.m_allocator);
				c.Reset(fixtureA, fixtureB);
				return c;
			}
			else
			{
				c = createFcn(this.m_allocator);
				c.Reset(fixtureB, fixtureA);
				return c;
			}
		}
		else
		{
			return null;
		}
	}
b2ContactFactory.prototype.Destroy = function (contact) {
		if (contact.m_manifold.m_pointCount > 0)
		{
			contact.m_fixtureA.m_body.SetAwake(true);
			contact.m_fixtureB.m_body.SetAwake(true);
		}
		
		var type1 = contact.m_fixtureA.GetType();
		var type2 = contact.m_fixtureB.GetType();
		
		
		
		
		var reg = this.m_registers[type1][type2];
		
		if (true)
		{
			reg.poolCount++;
			contact.m_next = reg.pool;
			reg.pool = contact;
		}
		
		var destroyFcn = reg.destroyFcn;
		destroyFcn(contact, this.m_allocator);
	}