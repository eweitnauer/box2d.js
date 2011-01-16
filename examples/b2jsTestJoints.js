(function(){

var Test = function() {
	b2jsTest.__constructor.apply(this, arguments);
};
extend(Test.prototype, b2jsTest.prototype)

Test.prototype.createWorld = function() {
	var world = b2jsTest.prototype.createWorld.apply(this, arguments);
	
	var m_physScale = 10;
	
	var ground = world.GetGroundBody();
	
	var body;
	var circleBody;
	var sd;
	var bd;
	var fixtureDef = new b2FixtureDef();
	
	//
	// CRANK
	//
	{
		// Define crank.
		sd = new b2PolygonShape();
		sd.SetAsBox(7.5 / m_physScale, 30.0 / m_physScale);
		fixtureDef.shape = sd;
		fixtureDef.density = 1.0;
		
		var rjd = new b2RevoluteJointDef();
		
		var prevBody = ground;
		
		bd = new b2BodyDef();
		bd.type = b2Body.b2_dynamicBody;
		bd.userData = "crank";
		bd.position.Set(100.0/m_physScale, (125.0)/m_physScale);
		body = world.CreateBody(bd);
		body.CreateFixture(fixtureDef);
		
		rjd.Initialize(prevBody, body, new b2Vec2(100.0/m_physScale, (95.0)/m_physScale));
		rjd.motorSpeed = 1.0 * -Math.PI;
		rjd.maxMotorTorque = 4400.0;
		rjd.enableMotor = true;
		m_joint1 = world.CreateJoint(rjd);
		
		prevBody = body;
		
		// Define follower.
		
		sd = new b2PolygonShape;
		sd.SetAsBox(7.5 / m_physScale, 60.0 / m_physScale);
		sd.userData = "follower";
		fixtureDef.shape = sd;
		bd.position.Set(100.0/m_physScale, (215.0)/m_physScale);
		body = world.CreateBody(bd);
		body.CreateFixture(fixtureDef);
		
		rjd.Initialize(prevBody, body, new b2Vec2(100.0/m_physScale, (155.0)/m_physScale));
		rjd.enableMotor = false;
		world.CreateJoint(rjd);
		
		prevBody = body;
		
		// Define piston
		sd = new b2PolygonShape();
		sd.SetAsBox(22.5 / m_physScale, 22.5 / m_physScale);
		fixtureDef.shape = sd;
		bd.position.Set(100.0/m_physScale, (275.0)/m_physScale);
		body = world.CreateBody(bd);
		body.CreateFixture(fixtureDef);
		
		rjd.Initialize(prevBody, body, new b2Vec2(100.0/m_physScale, (275.0)/m_physScale));
		world.CreateJoint(rjd);
		
		var pjd = new b2PrismaticJointDef();
		pjd.Initialize(ground, body, new b2Vec2(100.0/m_physScale, (275.0)/m_physScale), new b2Vec2(0.0, 1.0));
		
		pjd.maxMotorForce = 200.0;
		pjd.enableMotor = true;
		
		m_joint2 = world.CreateJoint(pjd);
		
		
		// Create a payload
		
		sd = new b2PolygonShape()
		sd.SetAsBox(12.5 / m_physScale, 12.5 / m_physScale);
		fixtureDef.shape = sd;
		fixtureDef.density = 2.0;
		bd.position.Set(110.0/m_physScale, (345.0)/m_physScale);
		body = world.CreateBody(bd);
		body.CreateFixture(fixtureDef);
		
	}
	
	return world;
};

window.b2jsTestJoints = Test;
	
})();