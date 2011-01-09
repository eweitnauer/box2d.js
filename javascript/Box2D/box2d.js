function extend(a, b) {
  for(var c in b) {
    a[c] = b[c]
  }
}
;var b2BoundValues = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2BoundValues.prototype.__constructor = function() {
  this.lowerValues = new Array;
  this.lowerValues[0] = 0;
  this.lowerValues[1] = 0;
  this.upperValues = new Array;
  this.upperValues[0] = 0;
  this.upperValues[1] = 0
};
b2BoundValues.prototype.__varz = function() {
};
b2BoundValues.prototype.lowerValues = null;
b2BoundValues.prototype.upperValues = null;var b2PairManager = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2PairManager.prototype.__constructor = function() {
  m_pairs = new Array;
  this.m_pairBuffer = new Array;
  m_pairCount = 0;
  this.m_pairBufferCount = 0;
  this.m_freePair = null
};
b2PairManager.prototype.__varz = function() {
};
b2PairManager.prototype.m_broadPhase = null;
b2PairManager.prototype.m_freePair = null;
b2PairManager.prototype.m_pairBuffer = null;
b2PairManager.prototype.m_pairBufferCount = 0;
b2PairManager.prototype.AddPair = function(proxy1, proxy2) {
  var pair = proxy1.pairs[proxy2];
  if(pair != null) {
    return pair
  }
  if(this.m_freePair == null) {
    this.m_freePair = new b2Pair;
    m_pairs.push(this.m_freePair)
  }
  pair = this.m_freePair;
  this.m_freePair = pair.next;
  pair.proxy1 = proxy1;
  pair.proxy2 = proxy2;
  pair.status = 0;
  pair.userData = null;
  pair.next = null;
  proxy1.pairs[proxy2] = pair;
  proxy2.pairs[proxy1] = pair;
  ++m_pairCount;
  return pair
};
b2PairManager.prototype.RemovePair = function(proxy1, proxy2) {
  var pair = proxy1.pairs[proxy2];
  if(pair == null) {
    return null
  }
  var userData = pair.userData;
  delete proxy1.pairs[proxy2];
  delete proxy2.pairs[proxy1];
  pair.next = this.m_freePair;
  pair.proxy1 = null;
  pair.proxy2 = null;
  pair.userData = null;
  pair.status = 0;
  this.m_freePair = pair;
  --m_pairCount;
  return userData
};
b2PairManager.prototype.Find = function(proxy1, proxy2) {
  return proxy1.pairs[proxy2]
};
b2PairManager.prototype.ValidateBuffer = function() {
};
b2PairManager.prototype.ValidateTable = function() {
};
b2PairManager.prototype.Initialize = function(broadPhase) {
  this.m_broadPhase = broadPhase
};
b2PairManager.prototype.AddBufferedPair = function(proxy1, proxy2) {
  var pair = this.AddPair(proxy1, proxy2);
  if(pair.IsBuffered() == false) {
    pair.SetBuffered();
    this.m_pairBuffer[this.m_pairBufferCount] = pair;
    ++this.m_pairBufferCount
  }
  pair.ClearRemoved();
  if(b2BroadPhase.s_validate) {
    this.ValidateBuffer()
  }
};
b2PairManager.prototype.RemoveBufferedPair = function(proxy1, proxy2) {
  var pair = this.Find(proxy1, proxy2);
  if(pair == null) {
    return
  }
  if(pair.IsBuffered() == false) {
    pair.SetBuffered();
    this.m_pairBuffer[this.m_pairBufferCount] = pair;
    ++this.m_pairBufferCount
  }
  pair.SetRemoved();
  if(b2BroadPhase.s_validate) {
    this.ValidateBuffer()
  }
};
b2PairManager.prototype.Commit = function(callback) {
  var i = 0;
  var removeCount = 0;
  for(i = 0;i < this.m_pairBufferCount;++i) {
    var pair = this.m_pairBuffer[i];
    pair.ClearBuffered();
    var proxy1 = pair.proxy1;
    var proxy2 = pair.proxy2;
    if(pair.IsRemoved()) {
    }else {
      if(pair.IsFinal() == false) {
        callback(proxy1.userData, proxy2.userData)
      }
    }
  }
  this.m_pairBufferCount = 0;
  if(b2BroadPhase.s_validate) {
    this.ValidateTable()
  }
};var b2TimeStep = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2TimeStep.prototype.__constructor = function() {
};
b2TimeStep.prototype.__varz = function() {
};
b2TimeStep.prototype.dt = null;
b2TimeStep.prototype.inv_dt = null;
b2TimeStep.prototype.dtRatio = null;
b2TimeStep.prototype.velocityIterations = 0;
b2TimeStep.prototype.positionIterations = 0;
b2TimeStep.prototype.warmStarting = null;
b2TimeStep.prototype.Set = function(step) {
  this.dt = step.dt;
  this.inv_dt = step.inv_dt;
  this.positionIterations = step.positionIterations;
  this.velocityIterations = step.velocityIterations;
  this.warmStarting = step.warmStarting
};var b2Controller = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Controller.prototype.__constructor = function() {
};
b2Controller.prototype.__varz = function() {
};
b2Controller.prototype.Step = function(step) {
};
b2Controller.prototype.Draw = function(debugDraw) {
};
b2Controller.prototype.AddBody = function(body) {
  var edge = new b2ControllerEdge;
  edge.controller = this;
  edge.body = body;
  edge.nextBody = m_bodyList;
  edge.prevBody = null;
  m_bodyList = edge;
  if(edge.nextBody) {
    edge.nextBody.prevBody = edge
  }
  m_bodyCount++;
  edge.nextController = body.m_controllerList;
  edge.prevController = null;
  body.m_controllerList = edge;
  if(edge.nextController) {
    edge.nextController.prevController = edge
  }
  body.m_controllerCount++
};
b2Controller.prototype.RemoveBody = function(body) {
  var edge = body.m_controllerList;
  while(edge && edge.controller != this) {
    edge = edge.nextController
  }
  if(edge.prevBody) {
    edge.prevBody.nextBody = edge.nextBody
  }
  if(edge.nextBody) {
    edge.nextBody.prevBody = edge.prevBody
  }
  if(edge.nextController) {
    edge.nextController.prevController = edge.prevController
  }
  if(edge.prevController) {
    edge.prevController.nextController = edge.nextController
  }
  if(m_bodyList == edge) {
    m_bodyList = edge.nextBody
  }
  if(body.m_controllerList == edge) {
    body.m_controllerList = edge.nextController
  }
  body.m_controllerCount--;
  m_bodyCount--
};
b2Controller.prototype.Clear = function() {
  while(m_bodyList) {
    this.RemoveBody(m_bodyList.body)
  }
};
b2Controller.prototype.GetNext = function() {
  return m_next
};
b2Controller.prototype.GetWorld = function() {
  return m_world
};
b2Controller.prototype.GetBodyList = function() {
  return m_bodyList
};var b2GravityController = function() {
  b2Controller.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2GravityController.prototype, b2Controller.prototype);
b2GravityController.prototype._super = function() {
  b2Controller.prototype.__constructor.apply(this, arguments)
};
b2GravityController.prototype.__constructor = function() {
};
b2GravityController.prototype.__varz = function() {
};
b2GravityController.prototype.G = 1;
b2GravityController.prototype.invSqr = true;
b2GravityController.prototype.Step = function(step) {
  var i = null;
  var body1 = null;
  var p1 = null;
  var mass1 = 0;
  var j = null;
  var body2 = null;
  var p2 = null;
  var dx = 0;
  var dy = 0;
  var r2 = 0;
  var f = null;
  if(this.invSqr) {
    for(i = m_bodyList;i;i = i.nextBody) {
      body1 = i.body;
      p1 = body1.GetWorldCenter();
      mass1 = body1.GetMass();
      for(j = m_bodyList;j != i;j = j.nextBody) {
        body2 = j.body;
        p2 = body2.GetWorldCenter();
        dx = p2.x - p1.x;
        dy = p2.y - p1.y;
        r2 = dx * dx + dy * dy;
        if(r2 < Number.MIN_VALUE) {
          continue
        }
        f = new b2Vec2(dx, dy);
        f.Multiply(this.G / r2 / Math.sqrt(r2) * mass1 * body2.GetMass());
        if(body1.IsAwake()) {
          body1.ApplyForce(f, p1)
        }
        f.Multiply(-1);
        if(body2.IsAwake()) {
          body2.ApplyForce(f, p2)
        }
      }
    }
  }else {
    for(i = m_bodyList;i;i = i.nextBody) {
      body1 = i.body;
      p1 = body1.GetWorldCenter();
      mass1 = body1.GetMass();
      for(j = m_bodyList;j != i;j = j.nextBody) {
        body2 = j.body;
        p2 = body2.GetWorldCenter();
        dx = p2.x - p1.x;
        dy = p2.y - p1.y;
        r2 = dx * dx + dy * dy;
        if(r2 < Number.MIN_VALUE) {
          continue
        }
        f = new b2Vec2(dx, dy);
        f.Multiply(this.G / r2 * mass1 * body2.GetMass());
        if(body1.IsAwake()) {
          body1.ApplyForce(f, p1)
        }
        f.Multiply(-1);
        if(body2.IsAwake()) {
          body2.ApplyForce(f, p2)
        }
      }
    }
  }
};var b2DestructionListener = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2DestructionListener.prototype.__constructor = function() {
};
b2DestructionListener.prototype.__varz = function() {
};
b2DestructionListener.prototype.SayGoodbyeJoint = function(joint) {
};
b2DestructionListener.prototype.SayGoodbyeFixture = function(fixture) {
};var b2ContactEdge = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ContactEdge.prototype.__constructor = function() {
};
b2ContactEdge.prototype.__varz = function() {
};
b2ContactEdge.prototype.other = null;
b2ContactEdge.prototype.contact = null;
b2ContactEdge.prototype.prev = null;
b2ContactEdge.prototype.next = null;var b2EdgeChainDef = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2EdgeChainDef.prototype.__constructor = function() {
  this.vertexCount = 0;
  this.isALoop = true;
  this.vertices = []
};
b2EdgeChainDef.prototype.__varz = function() {
};
b2EdgeChainDef.prototype.vertices = null;
b2EdgeChainDef.prototype.vertexCount = null;
b2EdgeChainDef.prototype.isALoop = null;var b2Vec2 = function(x_, y_) {
  if(arguments.length == 2) {
    this.x = x_;
    this.y = y_
  }
};
b2Vec2.Make = function(x_, y_) {
  return new b2Vec2(x_, y_)
};
b2Vec2.prototype.x = 0;
b2Vec2.prototype.y = 0;
b2Vec2.prototype.SetZero = function() {
  this.x = 0;
  this.y = 0
};
b2Vec2.prototype.Set = function(x_, y_) {
  this.x = x_;
  this.y = y_
};
b2Vec2.prototype.SetV = function(v) {
  this.x = v.x;
  this.y = v.y
};
b2Vec2.prototype.GetNegative = function() {
  return new b2Vec2(-this.x, -this.y)
};
b2Vec2.prototype.NegativeSelf = function() {
  this.x = -this.x;
  this.y = -this.y
};
b2Vec2.prototype.Copy = function() {
  return new b2Vec2(this.x, this.y)
};
b2Vec2.prototype.Add = function(v) {
  this.x += v.x;
  this.y += v.y
};
b2Vec2.prototype.Subtract = function(v) {
  this.x -= v.x;
  this.y -= v.y
};
b2Vec2.prototype.Multiply = function(a) {
  this.x *= a;
  this.y *= a
};
b2Vec2.prototype.MulM = function(A) {
  var tX = this.x;
  this.x = A.col1.x * tX + A.col2.x * this.y;
  this.y = A.col1.y * tX + A.col2.y * this.y
};
b2Vec2.prototype.MulTM = function(A) {
  var tX = b2Math.Dot(this, A.col1);
  this.y = b2Math.Dot(this, A.col2);
  this.x = tX
};
b2Vec2.prototype.CrossVF = function(s) {
  var tX = this.x;
  this.x = s * this.y;
  this.y = -s * tX
};
b2Vec2.prototype.CrossFV = function(s) {
  var tX = this.x;
  this.x = -s * this.y;
  this.y = s * tX
};
b2Vec2.prototype.MinV = function(b) {
  this.x = this.x < b.x ? this.x : b.x;
  this.y = this.y < b.y ? this.y : b.y
};
b2Vec2.prototype.MaxV = function(b) {
  this.x = this.x > b.x ? this.x : b.x;
  this.y = this.y > b.y ? this.y : b.y
};
b2Vec2.prototype.Abs = function() {
  if(this.x < 0) {
    this.x = -this.x
  }
  if(this.y < 0) {
    this.y = -this.y
  }
};
b2Vec2.prototype.Length = function() {
  return Math.sqrt(this.x * this.x + this.y * this.y)
};
b2Vec2.prototype.LengthSquared = function() {
  return this.x * this.x + this.y * this.y
};
b2Vec2.prototype.Normalize = function() {
  var length = Math.sqrt(this.x * this.x + this.y * this.y);
  if(length < Number.MIN_VALUE) {
    return 0
  }
  var invLength = 1 / length;
  this.x *= invLength;
  this.y *= invLength;
  return length
};
b2Vec2.prototype.IsValid = function() {
  return b2Math.IsValid(this.x) && b2Math.IsValid(this.y)
};var b2Vec3 = function(x, y, z) {
  if(arguments.length == 3) {
    this.x = x;
    this.y = y;
    this.z = z
  }
};
b2Vec3.prototype.x = 0;
b2Vec3.prototype.y = 0;
b2Vec3.prototype.z = 0;
b2Vec3.prototype.SetZero = function() {
  this.x = this.y = this.z = 0
};
b2Vec3.prototype.Set = function(x, y, z) {
  this.x = x;
  this.y = y;
  this.z = z
};
b2Vec3.prototype.SetV = function(v) {
  this.x = v.x;
  this.y = v.y;
  this.z = v.z
};
b2Vec3.prototype.GetNegative = function() {
  return new b2Vec3(-this.x, -this.y, -this.z)
};
b2Vec3.prototype.NegativeSelf = function() {
  this.x = -this.x;
  this.y = -this.y;
  this.z = -this.z
};
b2Vec3.prototype.Copy = function() {
  return new b2Vec3(this.x, this.y, this.z)
};
b2Vec3.prototype.Add = function(v) {
  this.x += v.x;
  this.y += v.y;
  this.z += v.z
};
b2Vec3.prototype.Subtract = function(v) {
  this.x -= v.x;
  this.y -= v.y;
  this.z -= v.z
};
b2Vec3.prototype.Multiply = function(a) {
  this.x *= a;
  this.y *= a;
  this.z *= a
};var b2DistanceProxy = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2DistanceProxy.prototype.__constructor = function() {
};
b2DistanceProxy.prototype.__varz = function() {
};
b2DistanceProxy.prototype.m_vertices = null;
b2DistanceProxy.prototype.m_count = 0;
b2DistanceProxy.prototype.m_radius = null;
b2DistanceProxy.prototype.Set = function(shape) {
  switch(shape.GetType()) {
    case b2Shape.e_circleShape:
      var circle = shape;
      this.m_vertices = new Array(1);
      this.m_vertices[0] = circle.m_p;
      this.m_count = 1;
      this.m_radius = circle.m_radius;
      break;
    case b2Shape.e_polygonShape:
      var polygon = shape;
      this.m_vertices = polygon.m_vertices;
      this.m_count = polygon.m_vertexCount;
      this.m_radius = polygon.m_radius;
      break;
    default:
      b2Settings.b2Assert(false)
  }
};
b2DistanceProxy.prototype.GetSupport = function(d) {
  var bestIndex = 0;
  var bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
  for(var i = 1;i < this.m_count;++i) {
    var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
    if(value > bestValue) {
      bestIndex = i;
      bestValue = value
    }
  }
  return bestIndex
};
b2DistanceProxy.prototype.GetSupportVertex = function(d) {
  var bestIndex = 0;
  var bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
  for(var i = 1;i < this.m_count;++i) {
    var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
    if(value > bestValue) {
      bestIndex = i;
      bestValue = value
    }
  }
  return this.m_vertices[bestIndex]
};
b2DistanceProxy.prototype.GetVertexCount = function() {
  return this.m_count
};
b2DistanceProxy.prototype.GetVertex = function(index) {
  b2Settings.b2Assert(0 <= index && index < this.m_count);
  return this.m_vertices[index]
};var b2ContactFactory = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ContactFactory.prototype.__constructor = function() {
};
b2ContactFactory.prototype.__varz = function() {
};
b2ContactFactory.prototype.m_registers = null;
b2ContactFactory.prototype.m_allocator = null;
b2ContactFactory.prototype.Create = function(fixtureA, fixtureB) {
  var type1 = fixtureA.GetType();
  var type2 = fixtureB.GetType();
  var reg = this.m_registers[type1][type2];
  var c;
  if(reg.pool) {
    c = reg.pool;
    reg.pool = c.m_next;
    reg.poolCount--;
    c.Reset(fixtureA, fixtureB);
    return c
  }
  var createFcn = reg.createFcn;
  if(createFcn != null) {
    if(reg.primary) {
      c = createFcn(this.m_allocator);
      c.Reset(fixtureA, fixtureB);
      return c
    }else {
      c = createFcn(this.m_allocator);
      c.Reset(fixtureB, fixtureA);
      return c
    }
  }else {
    return null
  }
};
b2ContactFactory.prototype.Destroy = function(contact) {
  if(contact.m_manifold.m_pointCount > 0) {
    contact.m_fixtureA.m_body.SetAwake(true);
    contact.m_fixtureB.m_body.SetAwake(true)
  }
  var type1 = contact.m_fixtureA.GetType();
  var type2 = contact.m_fixtureB.GetType();
  var reg = this.m_registers[type1][type2];
  if(true) {
    reg.poolCount++;
    contact.m_next = reg.pool;
    reg.pool = contact
  }
  var destroyFcn = reg.destroyFcn;
  destroyFcn(contact, this.m_allocator)
};var b2ConstantAccelController = function() {
  b2Controller.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2ConstantAccelController.prototype, b2Controller.prototype);
b2ConstantAccelController.prototype._super = function() {
  b2Controller.prototype.__constructor.apply(this, arguments)
};
b2ConstantAccelController.prototype.__constructor = function() {
};
b2ConstantAccelController.prototype.__varz = function() {
  this.A = new b2Vec2(0, 0)
};
b2ConstantAccelController.prototype.A = new b2Vec2(0, 0);
b2ConstantAccelController.prototype.Step = function(step) {
  var smallA = new b2Vec2(this.A.x * step.dt, this.A.y * step.dt);
  for(var i = m_bodyList;i;i = i.nextBody) {
    var body = i.body;
    if(!body.IsAwake()) {
      continue
    }
    body.SetLinearVelocity(new b2Vec2(body.GetLinearVelocity().x + smallA.x, body.GetLinearVelocity().y + smallA.y))
  }
};var b2SeparationFunction = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2SeparationFunction.prototype.__constructor = function() {
};
b2SeparationFunction.prototype.__varz = function() {
  this.m_localPoint = new b2Vec2;
  this.m_axis = new b2Vec2
};
b2SeparationFunction.e_points = 1;
b2SeparationFunction.e_faceA = 2;
b2SeparationFunction.e_faceB = 4;
b2SeparationFunction.prototype.m_proxyA = null;
b2SeparationFunction.prototype.m_proxyB = null;
b2SeparationFunction.prototype.m_type = 0;
b2SeparationFunction.prototype.m_localPoint = new b2Vec2;
b2SeparationFunction.prototype.m_axis = new b2Vec2;
b2SeparationFunction.prototype.Initialize = function(cache, proxyA, transformA, proxyB, transformB) {
  this.m_proxyA = proxyA;
  this.m_proxyB = proxyB;
  var count = cache.count;
  b2Settings.b2Assert(0 < count && count < 3);
  var localPointA;
  var localPointA1;
  var localPointA2;
  var localPointB;
  var localPointB1;
  var localPointB2;
  var pointAX;
  var pointAY;
  var pointBX;
  var pointBY;
  var normalX;
  var normalY;
  var tMat;
  var tVec;
  var s;
  var sgn;
  if(count == 1) {
    this.m_type = b2SeparationFunction.e_points;
    localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
    localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
    tVec = localPointA;
    tMat = transformA.R;
    pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    tVec = localPointB;
    tMat = transformB.R;
    pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    this.m_axis.x = pointBX - pointAX;
    this.m_axis.y = pointBY - pointAY;
    this.m_axis.Normalize()
  }else {
    if(cache.indexB[0] == cache.indexB[1]) {
      this.m_type = b2SeparationFunction.e_faceA;
      localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
      localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
      localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
      this.m_localPoint.x = 0.5 * (localPointA1.x + localPointA2.x);
      this.m_localPoint.y = 0.5 * (localPointA1.y + localPointA2.y);
      this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1);
      this.m_axis.Normalize();
      tVec = this.m_axis;
      tMat = transformA.R;
      normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      tVec = this.m_localPoint;
      tMat = transformA.R;
      pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tVec = localPointB;
      tMat = transformB.R;
      pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      s = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
      if(s < 0) {
        this.m_axis.NegativeSelf()
      }
    }else {
      if(cache.indexA[0] == cache.indexA[0]) {
        this.m_type = b2SeparationFunction.e_faceB;
        localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
        localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
        localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
        this.m_localPoint.x = 0.5 * (localPointB1.x + localPointB2.x);
        this.m_localPoint.y = 0.5 * (localPointB1.y + localPointB2.y);
        this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1);
        this.m_axis.Normalize();
        tVec = this.m_axis;
        tMat = transformB.R;
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tVec = this.m_localPoint;
        tMat = transformB.R;
        pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        tVec = localPointA;
        tMat = transformA.R;
        pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        s = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
        if(s < 0) {
          this.m_axis.NegativeSelf()
        }
      }else {
        localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
        localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
        localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
        localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
        var pA = b2Math.MulX(transformA, localPointA);
        var dA = b2Math.MulMV(transformA.R, b2Math.SubtractVV(localPointA2, localPointA1));
        var pB = b2Math.MulX(transformB, localPointB);
        var dB = b2Math.MulMV(transformB.R, b2Math.SubtractVV(localPointB2, localPointB1));
        var a = dA.x * dA.x + dA.y * dA.y;
        var e = dB.x * dB.x + dB.y * dB.y;
        var r = b2Math.SubtractVV(dB, dA);
        var c = dA.x * r.x + dA.y * r.y;
        var f = dB.x * r.x + dB.y * r.y;
        var b = dA.x * dB.x + dA.y * dB.y;
        var denom = a * e - b * b;
        s = 0;
        if(denom != 0) {
          s = b2Math.Clamp((b * f - c * e) / denom, 0, 1)
        }
        var t = (b * s + f) / e;
        if(t < 0) {
          t = 0;
          s = b2Math.Clamp((b - c) / a, 0, 1)
        }
        localPointA = new b2Vec2;
        localPointA.x = localPointA1.x + s * (localPointA2.x - localPointA1.x);
        localPointA.y = localPointA1.y + s * (localPointA2.y - localPointA1.y);
        localPointB = new b2Vec2;
        localPointB.x = localPointB1.x + s * (localPointB2.x - localPointB1.x);
        localPointB.y = localPointB1.y + s * (localPointB2.y - localPointB1.y);
        if(s == 0 || s == 1) {
          this.m_type = b2SeparationFunction.e_faceB;
          this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1);
          this.m_axis.Normalize();
          this.m_localPoint = localPointB;
          tVec = this.m_axis;
          tMat = transformB.R;
          normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
          normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
          tVec = this.m_localPoint;
          tMat = transformB.R;
          pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
          pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
          tVec = localPointA;
          tMat = transformA.R;
          pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
          pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
          sgn = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
          if(s < 0) {
            this.m_axis.NegativeSelf()
          }
        }else {
          this.m_type = b2SeparationFunction.e_faceA;
          this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1);
          this.m_localPoint = localPointA;
          tVec = this.m_axis;
          tMat = transformA.R;
          normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
          normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
          tVec = this.m_localPoint;
          tMat = transformA.R;
          pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
          pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
          tVec = localPointB;
          tMat = transformB.R;
          pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
          pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
          sgn = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
          if(s < 0) {
            this.m_axis.NegativeSelf()
          }
        }
      }
    }
  }
};
b2SeparationFunction.prototype.Evaluate = function(transformA, transformB) {
  var axisA;
  var axisB;
  var localPointA;
  var localPointB;
  var pointA;
  var pointB;
  var seperation;
  var normal;
  switch(this.m_type) {
    case b2SeparationFunction.e_points:
      axisA = b2Math.MulTMV(transformA.R, this.m_axis);
      axisB = b2Math.MulTMV(transformB.R, this.m_axis.GetNegative());
      localPointA = this.m_proxyA.GetSupportVertex(axisA);
      localPointB = this.m_proxyB.GetSupportVertex(axisB);
      pointA = b2Math.MulX(transformA, localPointA);
      pointB = b2Math.MulX(transformB, localPointB);
      seperation = (pointB.x - pointA.x) * this.m_axis.x + (pointB.y - pointA.y) * this.m_axis.y;
      return seperation;
    case b2SeparationFunction.e_faceA:
      normal = b2Math.MulMV(transformA.R, this.m_axis);
      pointA = b2Math.MulX(transformA, this.m_localPoint);
      axisB = b2Math.MulTMV(transformB.R, normal.GetNegative());
      localPointB = this.m_proxyB.GetSupportVertex(axisB);
      pointB = b2Math.MulX(transformB, localPointB);
      seperation = (pointB.x - pointA.x) * normal.x + (pointB.y - pointA.y) * normal.y;
      return seperation;
    case b2SeparationFunction.e_faceB:
      normal = b2Math.MulMV(transformB.R, this.m_axis);
      pointB = b2Math.MulX(transformB, this.m_localPoint);
      axisA = b2Math.MulTMV(transformA.R, normal.GetNegative());
      localPointA = this.m_proxyA.GetSupportVertex(axisA);
      pointA = b2Math.MulX(transformA, localPointA);
      seperation = (pointA.x - pointB.x) * normal.x + (pointA.y - pointB.y) * normal.y;
      return seperation;
    default:
      b2Settings.b2Assert(false);
      return 0
  }
};var b2DynamicTreePair = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2DynamicTreePair.prototype.__constructor = function() {
};
b2DynamicTreePair.prototype.__varz = function() {
};
b2DynamicTreePair.prototype.proxyA = null;
b2DynamicTreePair.prototype.proxyB = null;var b2ContactConstraintPoint = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ContactConstraintPoint.prototype.__constructor = function() {
};
b2ContactConstraintPoint.prototype.__varz = function() {
  this.localPoint = new b2Vec2;
  this.rA = new b2Vec2;
  this.rB = new b2Vec2
};
b2ContactConstraintPoint.prototype.localPoint = new b2Vec2;
b2ContactConstraintPoint.prototype.rA = new b2Vec2;
b2ContactConstraintPoint.prototype.rB = new b2Vec2;
b2ContactConstraintPoint.prototype.normalImpulse = null;
b2ContactConstraintPoint.prototype.tangentImpulse = null;
b2ContactConstraintPoint.prototype.normalMass = null;
b2ContactConstraintPoint.prototype.tangentMass = null;
b2ContactConstraintPoint.prototype.equalizedMass = null;
b2ContactConstraintPoint.prototype.velocityBias = null;var b2ContactFilter = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ContactFilter.prototype.__constructor = function() {
};
b2ContactFilter.prototype.__varz = function() {
};
b2ContactFilter.prototype.ShouldCollide = function(fixtureA, fixtureB) {
  var filter1 = fixtureA.GetFilterData();
  var filter2 = fixtureB.GetFilterData();
  if(filter1.groupIndex == filter2.groupIndex && filter1.groupIndex != 0) {
    return filter1.groupIndex > 0
  }
  var collide = (filter1.maskBits & filter2.categoryBits) != 0 && (filter1.categoryBits & filter2.maskBits) != 0;
  return collide
};
b2ContactFilter.prototype.RayCollide = function(userData, fixture) {
  if(!userData) {
    return true
  }
  return this.ShouldCollide(userData, fixture)
};var b2ControllerEdge = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ControllerEdge.prototype.__constructor = function() {
};
b2ControllerEdge.prototype.__varz = function() {
};
b2ControllerEdge.prototype.controller = null;
b2ControllerEdge.prototype.body = null;
b2ControllerEdge.prototype.prevBody = null;
b2ControllerEdge.prototype.nextBody = null;
b2ControllerEdge.prototype.prevController = null;
b2ControllerEdge.prototype.nextController = null;var b2DistanceInput = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2DistanceInput.prototype.__constructor = function() {
};
b2DistanceInput.prototype.__varz = function() {
};
b2DistanceInput.prototype.proxyA = null;
b2DistanceInput.prototype.proxyB = null;
b2DistanceInput.prototype.transformA = null;
b2DistanceInput.prototype.transformB = null;
b2DistanceInput.prototype.useRadii = null;var b2Settings = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Settings.prototype.__constructor = function() {
};
b2Settings.prototype.__varz = function() {
};
b2Settings.VERSION = "2.1alpha";
b2Settings.USHRT_MAX = 65535;
b2Settings.b2_pi = Math.PI;
b2Settings.b2_maxManifoldPoints = 2;
b2Settings.b2_aabbExtension = 0.1;
b2Settings.b2_aabbMultiplier = 2;
b2Settings.b2_polygonRadius = 2 * b2Settings.b2_linearSlop;
b2Settings.b2_linearSlop = 0.0050;
b2Settings.b2_angularSlop = 2 / 180 * b2Settings.b2_pi;
b2Settings.b2_toiSlop = 8 * b2Settings.b2_linearSlop;
b2Settings.b2_maxTOIContactsPerIsland = 32;
b2Settings.b2_maxTOIJointsPerIsland = 32;
b2Settings.b2_velocityThreshold = 1;
b2Settings.b2_maxLinearCorrection = 0.2;
b2Settings.b2_maxAngularCorrection = 8 / 180 * b2Settings.b2_pi;
b2Settings.b2_maxTranslation = 2;
b2Settings.b2_maxTranslationSquared = b2Settings.b2_maxTranslation * b2Settings.b2_maxTranslation;
b2Settings.b2_maxRotation = 0.5 * b2Settings.b2_pi;
b2Settings.b2_maxRotationSquared = b2Settings.b2_maxRotation * b2Settings.b2_maxRotation;
b2Settings.b2_contactBaumgarte = 0.2;
b2Settings.b2_timeToSleep = 0.5;
b2Settings.b2_linearSleepTolerance = 0.01;
b2Settings.b2_angularSleepTolerance = 2 / 180 * b2Settings.b2_pi;
b2Settings.b2MixFriction = function(friction1, friction2) {
  return Math.sqrt(friction1 * friction2)
};
b2Settings.b2MixRestitution = function(restitution1, restitution2) {
  return restitution1 > restitution2 ? restitution1 : restitution2
};
b2Settings.b2Assert = function(a) {
  if(!a) {
    throw"Assertion Failed";
  }
};var b2Proxy = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Proxy.prototype.__constructor = function() {
};
b2Proxy.prototype.__varz = function() {
  this.lowerBounds = new Array(2);
  this.upperBounds = new Array(2);
  this.pairs = new Dictionary
};
b2Proxy.prototype.lowerBounds = new Array(2);
b2Proxy.prototype.upperBounds = new Array(2);
b2Proxy.prototype.overlapCount = 0;
b2Proxy.prototype.timeStamp = 0;
b2Proxy.prototype.pairs = new Dictionary;
b2Proxy.prototype.next = null;
b2Proxy.prototype.userData = null;
b2Proxy.prototype.IsValid = function() {
  return this.overlapCount != b2BroadPhase.b2_invalid
};var b2Point = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Point.prototype.__constructor = function() {
};
b2Point.prototype.__varz = function() {
  this.p = new b2Vec2
};
b2Point.prototype.p = new b2Vec2;
b2Point.prototype.Support = function(xf, vX, vY) {
  return this.p
};
b2Point.prototype.GetFirstVertex = function(xf) {
  return this.p
};var b2BroadPhase = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2BroadPhase.prototype.__constructor = function(worldAABB) {
  var i = 0;
  m_pairManager.Initialize(this);
  m_worldAABB = worldAABB;
  m_proxyCount = 0;
  m_bounds = new Array;
  for(i = 0;i < 2;i++) {
    m_bounds[i] = new Array
  }
  var dX = worldAABB.upperBound.x - worldAABB.lowerBound.x;
  var dY = worldAABB.upperBound.y - worldAABB.lowerBound.y;
  m_quantizationFactor.x = b2Settings.USHRT_MAX / dX;
  m_quantizationFactor.y = b2Settings.USHRT_MAX / dY;
  this.m_timeStamp = 1;
  this.m_queryResultCount = 0
};
b2BroadPhase.prototype.__varz = function() {
  this.m_querySortKeys = new Array;
  this.m_queryResults = new Array
};
b2BroadPhase.s_validate = false;
b2BroadPhase.b2_invalid = b2Settings.USHRT_MAX;
b2BroadPhase.b2_nullEdge = b2Settings.USHRT_MAX;
b2BroadPhase.BinarySearch = function(bounds, count, value) {
  var low = 0;
  var high = count - 1;
  while(low <= high) {
    var mid = Math.round((low + high) / 2);
    var bound = bounds[mid];
    if(bound.value > value) {
      high = mid - 1
    }else {
      if(bound.value < value) {
        low = mid + 1
      }else {
        return parseInt(mid)
      }
    }
  }
  return parseInt(low)
};
b2BroadPhase.prototype.m_freeProxy = null;
b2BroadPhase.prototype.m_querySortKeys = new Array;
b2BroadPhase.prototype.m_queryResults = new Array;
b2BroadPhase.prototype.m_queryResultCount = 0;
b2BroadPhase.prototype.m_timeStamp = 0;
b2BroadPhase.prototype.ComputeBounds = function(lowerValues, upperValues, aabb) {
  var minVertexX = aabb.lowerBound.x;
  var minVertexY = aabb.lowerBound.y;
  minVertexX = b2Math.Min(minVertexX, m_worldAABB.upperBound.x);
  minVertexY = b2Math.Min(minVertexY, m_worldAABB.upperBound.y);
  minVertexX = b2Math.Max(minVertexX, m_worldAABB.lowerBound.x);
  minVertexY = b2Math.Max(minVertexY, m_worldAABB.lowerBound.y);
  var maxVertexX = aabb.upperBound.x;
  var maxVertexY = aabb.upperBound.y;
  maxVertexX = b2Math.Min(maxVertexX, m_worldAABB.upperBound.x);
  maxVertexY = b2Math.Min(maxVertexY, m_worldAABB.upperBound.y);
  maxVertexX = b2Math.Max(maxVertexX, m_worldAABB.lowerBound.x);
  maxVertexY = b2Math.Max(maxVertexY, m_worldAABB.lowerBound.y);
  lowerValues[0] = parseInt(m_quantizationFactor.x * (minVertexX - m_worldAABB.lowerBound.x)) & b2Settings.USHRT_MAX - 1;
  upperValues[0] = parseInt(m_quantizationFactor.x * (maxVertexX - m_worldAABB.lowerBound.x)) % 65535 | 1;
  lowerValues[1] = parseInt(m_quantizationFactor.y * (minVertexY - m_worldAABB.lowerBound.y)) & b2Settings.USHRT_MAX - 1;
  upperValues[1] = parseInt(m_quantizationFactor.y * (maxVertexY - m_worldAABB.lowerBound.y)) % 65535 | 1
};
b2BroadPhase.prototype.TestOverlapValidate = function(p1, p2) {
  for(var axis = 0;axis < 2;++axis) {
    var bounds = m_bounds[axis];
    var bound1 = bounds[p1.lowerBounds[axis]];
    var bound2 = bounds[p2.upperBounds[axis]];
    if(bound1.value > bound2.value) {
      return false
    }
    bound1 = bounds[p1.upperBounds[axis]];
    bound2 = bounds[p2.lowerBounds[axis]];
    if(bound1.value < bound2.value) {
      return false
    }
  }
  return true
};
b2BroadPhase.prototype.QueryAxis = function(lowerQueryOut, upperQueryOut, lowerValue, upperValue, bounds, boundCount, axis) {
  var lowerQuery = b2BroadPhase.BinarySearch(bounds, boundCount, lowerValue);
  var upperQuery = b2BroadPhase.BinarySearch(bounds, boundCount, upperValue);
  var bound;
  for(var j = lowerQuery;j < upperQuery;++j) {
    bound = bounds[j];
    if(bound.IsLower()) {
      this.IncrementOverlapCount(bound.proxy)
    }
  }
  if(lowerQuery > 0) {
    var i = lowerQuery - 1;
    bound = bounds[i];
    var s = bound.stabbingCount;
    while(s) {
      bound = bounds[i];
      if(bound.IsLower()) {
        var proxy = bound.proxy;
        if(lowerQuery <= proxy.upperBounds[axis]) {
          this.IncrementOverlapCount(bound.proxy);
          --s
        }
      }
      --i
    }
  }
  lowerQueryOut[0] = lowerQuery;
  upperQueryOut[0] = upperQuery
};
b2BroadPhase.prototype.IncrementOverlapCount = function(proxy) {
  if(proxy.timeStamp < this.m_timeStamp) {
    proxy.timeStamp = this.m_timeStamp;
    proxy.overlapCount = 1
  }else {
    proxy.overlapCount = 2;
    this.m_queryResults[this.m_queryResultCount] = proxy;
    ++this.m_queryResultCount
  }
};
b2BroadPhase.prototype.IncrementTimeStamp = function() {
  if(this.m_timeStamp == b2Settings.USHRT_MAX) {
    for(var i = 0;i < m_proxyPool.length;++i) {
      m_proxyPool[i].timeStamp = 0
    }
    this.m_timeStamp = 1
  }else {
    ++this.m_timeStamp
  }
};
b2BroadPhase.prototype.InRange = function(aabb) {
  var dX;
  var dY;
  var d2X;
  var d2Y;
  dX = aabb.lowerBound.x;
  dY = aabb.lowerBound.y;
  dX -= m_worldAABB.upperBound.x;
  dY -= m_worldAABB.upperBound.y;
  d2X = m_worldAABB.lowerBound.x;
  d2Y = m_worldAABB.lowerBound.y;
  d2X -= aabb.upperBound.x;
  d2Y -= aabb.upperBound.y;
  dX = b2Math.Max(dX, d2X);
  dY = b2Math.Max(dY, d2Y);
  return b2Math.Max(dX, dY) < 0
};
b2BroadPhase.prototype.CreateProxy = function(aabb, userData) {
  var index = 0;
  var proxy;
  var i = 0;
  var j = 0;
  if(!this.m_freeProxy) {
    this.m_freeProxy = m_proxyPool[m_proxyCount] = new b2Proxy;
    this.m_freeProxy.next = null;
    this.m_freeProxy.timeStamp = 0;
    this.m_freeProxy.overlapCount = b2BroadPhase.b2_invalid;
    this.m_freeProxy.userData = null;
    for(i = 0;i < 2;i++) {
      j = m_proxyCount * 2;
      m_bounds[i][j++] = new b2Bound;
      m_bounds[i][j] = new b2Bound
    }
  }
  proxy = this.m_freeProxy;
  this.m_freeProxy = proxy.next;
  proxy.overlapCount = 0;
  proxy.userData = userData;
  var boundCount = 2 * m_proxyCount;
  var lowerValues = new Array;
  var upperValues = new Array;
  this.ComputeBounds(lowerValues, upperValues, aabb);
  for(var axis = 0;axis < 2;++axis) {
    var bounds = m_bounds[axis];
    var lowerIndex = 0;
    var upperIndex = 0;
    var lowerIndexOut = new Array;
    lowerIndexOut.push(lowerIndex);
    var upperIndexOut = new Array;
    upperIndexOut.push(upperIndex);
    this.QueryAxis(lowerIndexOut, upperIndexOut, lowerValues[axis], upperValues[axis], bounds, boundCount, axis);
    lowerIndex = lowerIndexOut[0];
    upperIndex = upperIndexOut[0];
    bounds.splice(upperIndex, 0, bounds[bounds.length - 1]);
    bounds.length--;
    bounds.splice(lowerIndex, 0, bounds[bounds.length - 1]);
    bounds.length--;
    ++upperIndex;
    var tBound1 = bounds[lowerIndex];
    var tBound2 = bounds[upperIndex];
    tBound1.value = lowerValues[axis];
    tBound1.proxy = proxy;
    tBound2.value = upperValues[axis];
    tBound2.proxy = proxy;
    var tBoundAS3 = bounds[parseInt(lowerIndex - 1)];
    tBound1.stabbingCount = lowerIndex == 0 ? 0 : tBoundAS3.stabbingCount;
    tBoundAS3 = bounds[parseInt(upperIndex - 1)];
    tBound2.stabbingCount = tBoundAS3.stabbingCount;
    for(index = lowerIndex;index < upperIndex;++index) {
      tBoundAS3 = bounds[index];
      tBoundAS3.stabbingCount++
    }
    for(index = lowerIndex;index < boundCount + 2;++index) {
      tBound1 = bounds[index];
      var proxy2 = tBound1.proxy;
      if(tBound1.IsLower()) {
        proxy2.lowerBounds[axis] = index
      }else {
        proxy2.upperBounds[axis] = index
      }
    }
  }
  ++m_proxyCount;
  for(i = 0;i < this.m_queryResultCount;++i) {
    m_pairManager.AddBufferedPair(proxy, this.m_queryResults[i])
  }
  this.m_queryResultCount = 0;
  this.IncrementTimeStamp();
  return proxy
};
b2BroadPhase.prototype.DestroyProxy = function(proxy_) {
  var proxy = proxy_;
  var tBound1;
  var tBound2;
  var boundCount = 2 * m_proxyCount;
  for(var axis = 0;axis < 2;++axis) {
    var bounds = m_bounds[axis];
    var lowerIndex = proxy.lowerBounds[axis];
    var upperIndex = proxy.upperBounds[axis];
    tBound1 = bounds[lowerIndex];
    var lowerValue = tBound1.value;
    tBound2 = bounds[upperIndex];
    var upperValue = tBound2.value;
    bounds.splice(upperIndex, 1);
    bounds.splice(lowerIndex, 1);
    bounds.push(tBound1);
    bounds.push(tBound2);
    var tEnd = boundCount - 2;
    for(var index = lowerIndex;index < tEnd;++index) {
      tBound1 = bounds[index];
      var proxy2 = tBound1.proxy;
      if(tBound1.IsLower()) {
        proxy2.lowerBounds[axis] = index
      }else {
        proxy2.upperBounds[axis] = index
      }
    }
    tEnd = upperIndex - 1;
    for(var index2 = lowerIndex;index2 < tEnd;++index2) {
      tBound1 = bounds[index2];
      tBound1.stabbingCount--
    }
    var ignore = new Array;
    this.QueryAxis(ignore, ignore, lowerValue, upperValue, bounds, boundCount - 2, axis)
  }
  for(var i = 0;i < this.m_queryResultCount;++i) {
    m_pairManager.RemoveBufferedPair(proxy, this.m_queryResults[i])
  }
  this.m_queryResultCount = 0;
  this.IncrementTimeStamp();
  proxy.userData = null;
  proxy.overlapCount = b2BroadPhase.b2_invalid;
  proxy.lowerBounds[0] = b2BroadPhase.b2_invalid;
  proxy.lowerBounds[1] = b2BroadPhase.b2_invalid;
  proxy.upperBounds[0] = b2BroadPhase.b2_invalid;
  proxy.upperBounds[1] = b2BroadPhase.b2_invalid;
  proxy.next = this.m_freeProxy;
  this.m_freeProxy = proxy;
  --m_proxyCount
};
b2BroadPhase.prototype.MoveProxy = function(proxy_, aabb, displacement) {
  var proxy = proxy_;
  var as3arr;
  var as3int = 0;
  var axis = 0;
  var index = 0;
  var bound;
  var prevBound;
  var nextBound;
  var nextProxyId = 0;
  var nextProxy;
  if(proxy == null) {
    return
  }
  if(aabb.IsValid() == false) {
    return
  }
  var boundCount = 2 * m_proxyCount;
  var newValues = new b2BoundValues;
  this.ComputeBounds(newValues.lowerValues, newValues.upperValues, aabb);
  var oldValues = new b2BoundValues;
  for(axis = 0;axis < 2;++axis) {
    bound = m_bounds[axis][proxy.lowerBounds[axis]];
    oldValues.lowerValues[axis] = bound.value;
    bound = m_bounds[axis][proxy.upperBounds[axis]];
    oldValues.upperValues[axis] = bound.value
  }
  for(axis = 0;axis < 2;++axis) {
    var bounds = m_bounds[axis];
    var lowerIndex = proxy.lowerBounds[axis];
    var upperIndex = proxy.upperBounds[axis];
    var lowerValue = newValues.lowerValues[axis];
    var upperValue = newValues.upperValues[axis];
    bound = bounds[lowerIndex];
    var deltaLower = lowerValue - bound.value;
    bound.value = lowerValue;
    bound = bounds[upperIndex];
    var deltaUpper = upperValue - bound.value;
    bound.value = upperValue;
    if(deltaLower < 0) {
      index = lowerIndex;
      while(index > 0 && lowerValue < bounds[parseInt(index - 1)].value) {
        bound = bounds[index];
        prevBound = bounds[parseInt(index - 1)];
        var prevProxy = prevBound.proxy;
        prevBound.stabbingCount++;
        if(prevBound.IsUpper() == true) {
          if(this.TestOverlapBound(newValues, prevProxy)) {
            m_pairManager.AddBufferedPair(proxy, prevProxy)
          }
          as3arr = prevProxy.upperBounds;
          as3int = as3arr[axis];
          as3int++;
          as3arr[axis] = as3int;
          bound.stabbingCount++
        }else {
          as3arr = prevProxy.lowerBounds;
          as3int = as3arr[axis];
          as3int++;
          as3arr[axis] = as3int;
          bound.stabbingCount--
        }
        as3arr = proxy.lowerBounds;
        as3int = as3arr[axis];
        as3int--;
        as3arr[axis] = as3int;
        bound.Swap(prevBound);
        --index
      }
    }
    if(deltaUpper > 0) {
      index = upperIndex;
      while(index < boundCount - 1 && bounds[parseInt(index + 1)].value <= upperValue) {
        bound = bounds[index];
        nextBound = bounds[parseInt(index + 1)];
        nextProxy = nextBound.proxy;
        nextBound.stabbingCount++;
        if(nextBound.IsLower() == true) {
          if(this.TestOverlapBound(newValues, nextProxy)) {
            m_pairManager.AddBufferedPair(proxy, nextProxy)
          }
          as3arr = nextProxy.lowerBounds;
          as3int = as3arr[axis];
          as3int--;
          as3arr[axis] = as3int;
          bound.stabbingCount++
        }else {
          as3arr = nextProxy.upperBounds;
          as3int = as3arr[axis];
          as3int--;
          as3arr[axis] = as3int;
          bound.stabbingCount--
        }
        as3arr = proxy.upperBounds;
        as3int = as3arr[axis];
        as3int++;
        as3arr[axis] = as3int;
        bound.Swap(nextBound);
        index++
      }
    }
    if(deltaLower > 0) {
      index = lowerIndex;
      while(index < boundCount - 1 && bounds[parseInt(index + 1)].value <= lowerValue) {
        bound = bounds[index];
        nextBound = bounds[parseInt(index + 1)];
        nextProxy = nextBound.proxy;
        nextBound.stabbingCount--;
        if(nextBound.IsUpper()) {
          if(this.TestOverlapBound(oldValues, nextProxy)) {
            m_pairManager.RemoveBufferedPair(proxy, nextProxy)
          }
          as3arr = nextProxy.upperBounds;
          as3int = as3arr[axis];
          as3int--;
          as3arr[axis] = as3int;
          bound.stabbingCount--
        }else {
          as3arr = nextProxy.lowerBounds;
          as3int = as3arr[axis];
          as3int--;
          as3arr[axis] = as3int;
          bound.stabbingCount++
        }
        as3arr = proxy.lowerBounds;
        as3int = as3arr[axis];
        as3int++;
        as3arr[axis] = as3int;
        bound.Swap(nextBound);
        index++
      }
    }
    if(deltaUpper < 0) {
      index = upperIndex;
      while(index > 0 && upperValue < bounds[parseInt(index - 1)].value) {
        bound = bounds[index];
        prevBound = bounds[parseInt(index - 1)];
        prevProxy = prevBound.proxy;
        prevBound.stabbingCount--;
        if(prevBound.IsLower() == true) {
          if(this.TestOverlapBound(oldValues, prevProxy)) {
            m_pairManager.RemoveBufferedPair(proxy, prevProxy)
          }
          as3arr = prevProxy.lowerBounds;
          as3int = as3arr[axis];
          as3int++;
          as3arr[axis] = as3int;
          bound.stabbingCount--
        }else {
          as3arr = prevProxy.upperBounds;
          as3int = as3arr[axis];
          as3int++;
          as3arr[axis] = as3int;
          bound.stabbingCount++
        }
        as3arr = proxy.upperBounds;
        as3int = as3arr[axis];
        as3int--;
        as3arr[axis] = as3int;
        bound.Swap(prevBound);
        index--
      }
    }
  }
};
b2BroadPhase.prototype.UpdatePairs = function(callback) {
  m_pairManager.Commit(callback)
};
b2BroadPhase.prototype.TestOverlap = function(proxyA, proxyB) {
  var proxyA_ = proxyA;
  var proxyB_ = proxyB;
  if(proxyA_.lowerBounds[0] > proxyB_.upperBounds[0]) {
    return false
  }
  if(proxyB_.lowerBounds[0] > proxyA_.upperBounds[0]) {
    return false
  }
  if(proxyA_.lowerBounds[1] > proxyB_.upperBounds[1]) {
    return false
  }
  if(proxyB_.lowerBounds[1] > proxyA_.upperBounds[1]) {
    return false
  }
  return true
};
b2BroadPhase.prototype.GetUserData = function(proxy) {
  return proxy.userData
};
b2BroadPhase.prototype.GetFatAABB = function(proxy_) {
  var aabb = new b2AABB;
  var proxy = proxy_;
  aabb.lowerBound.x = m_worldAABB.lowerBound.x + m_bounds[0][proxy.lowerBounds[0]].value / m_quantizationFactor.x;
  aabb.lowerBound.y = m_worldAABB.lowerBound.y + m_bounds[1][proxy.lowerBounds[1]].value / m_quantizationFactor.y;
  aabb.upperBound.x = m_worldAABB.lowerBound.x + m_bounds[0][proxy.upperBounds[0]].value / m_quantizationFactor.x;
  aabb.upperBound.y = m_worldAABB.lowerBound.y + m_bounds[1][proxy.upperBounds[1]].value / m_quantizationFactor.y;
  return aabb
};
b2BroadPhase.prototype.GetProxyCount = function() {
  return m_proxyCount
};
b2BroadPhase.prototype.Query = function(callback, aabb) {
  var lowerValues = new Array;
  var upperValues = new Array;
  this.ComputeBounds(lowerValues, upperValues, aabb);
  var lowerIndex = 0;
  var upperIndex = 0;
  var lowerIndexOut = new Array;
  lowerIndexOut.push(lowerIndex);
  var upperIndexOut = new Array;
  upperIndexOut.push(upperIndex);
  this.QueryAxis(lowerIndexOut, upperIndexOut, lowerValues[0], upperValues[0], m_bounds[0], 2 * m_proxyCount, 0);
  this.QueryAxis(lowerIndexOut, upperIndexOut, lowerValues[1], upperValues[1], m_bounds[1], 2 * m_proxyCount, 1);
  for(var i = 0;i < this.m_queryResultCount;++i) {
    var proxy = this.m_queryResults[i];
    if(!callback(proxy)) {
      break
    }
  }
  this.m_queryResultCount = 0;
  this.IncrementTimeStamp()
};
b2BroadPhase.prototype.Validate = function() {
  var pair;
  var proxy1;
  var proxy2;
  var overlap;
  for(var axis = 0;axis < 2;++axis) {
    var bounds = m_bounds[axis];
    var boundCount = 2 * m_proxyCount;
    var stabbingCount = 0;
    for(var i = 0;i < boundCount;++i) {
      var bound = bounds[i];
      if(bound.IsLower() == true) {
        stabbingCount++
      }else {
        stabbingCount--
      }
    }
  }
};
b2BroadPhase.prototype.Rebalance = function(iterations) {
};
b2BroadPhase.prototype.RayCast = function(callback, input) {
  var subInput = new b2RayCastInput;
  subInput.p1.SetV(input.p1);
  subInput.p2.SetV(input.p2);
  subInput.maxFraction = input.maxFraction;
  var dx = (input.p2.x - input.p1.x) * m_quantizationFactor.x;
  var dy = (input.p2.y - input.p1.y) * m_quantizationFactor.y;
  var sx = dx < -Number.MIN_VALUE ? -1 : dx > Number.MIN_VALUE ? 1 : 0;
  var sy = dy < -Number.MIN_VALUE ? -1 : dy > Number.MIN_VALUE ? 1 : 0;
  var p1x = m_quantizationFactor.x * (input.p1.x - m_worldAABB.lowerBound.x);
  var p1y = m_quantizationFactor.y * (input.p1.y - m_worldAABB.lowerBound.y);
  var startValues = new Array;
  var startValues2 = new Array;
  startValues[0] = parseInt(p1x) & b2Settings.USHRT_MAX - 1;
  startValues[1] = parseInt(p1y) & b2Settings.USHRT_MAX - 1;
  startValues2[0] = startValues[0] + 1;
  startValues2[1] = startValues[1] + 1;
  var startIndices = new Array;
  var xIndex = 0;
  var yIndex = 0;
  var proxy;
  var lowerIndex = 0;
  var upperIndex = 0;
  var lowerIndexOut = new Array;
  lowerIndexOut.push(lowerIndex);
  var upperIndexOut = new Array;
  upperIndexOut.push(upperIndex);
  this.QueryAxis(lowerIndexOut, upperIndexOut, startValues[0], startValues2[0], m_bounds[0], 2 * m_proxyCount, 0);
  if(sx >= 0) {
    xIndex = upperIndexOut[0] - 1
  }else {
    xIndex = lowerIndexOut[0]
  }
  this.QueryAxis(lowerIndexOut, upperIndexOut, startValues[1], startValues2[1], m_bounds[1], 2 * m_proxyCount, 1);
  if(sy >= 0) {
    yIndex = upperIndexOut[0] - 1
  }else {
    yIndex = lowerIndexOut[0]
  }
  for(var i = 0;i < this.m_queryResultCount;i++) {
    subInput.maxFraction = callback(this.m_queryResults[i], subInput)
  }
  for(;;) {
    var xProgress = 0;
    var yProgress = 0;
    xIndex += sx >= 0 ? 1 : -1;
    if(xIndex < 0 || xIndex >= m_proxyCount * 2) {
      break
    }
    if(sx != 0) {
      xProgress = (m_bounds[0][xIndex].value - p1x) / dx
    }
    yIndex += sy >= 0 ? 1 : -1;
    if(yIndex < 0 || yIndex >= m_proxyCount * 2) {
      break
    }
    if(sy != 0) {
      yProgress = (m_bounds[1][yIndex].value - p1y) / dy
    }
    for(;;) {
      if(sy == 0 || sx != 0 && xProgress < yProgress) {
        if(xProgress > subInput.maxFraction) {
          break
        }
        if(sx > 0 ? m_bounds[0][xIndex].IsLower() : m_bounds[0][xIndex].IsUpper()) {
          proxy = m_bounds[0][xIndex].proxy;
          if(sy >= 0) {
            if(proxy.lowerBounds[1] <= yIndex - 1 && proxy.upperBounds[1] >= yIndex) {
              subInput.maxFraction = callback(proxy, subInput)
            }
          }else {
            if(proxy.lowerBounds[1] <= yIndex && proxy.upperBounds[1] >= yIndex + 1) {
              subInput.maxFraction = callback(proxy, subInput)
            }
          }
        }
        if(subInput.maxFraction == 0) {
          break
        }
        if(sx > 0) {
          xIndex++;
          if(xIndex == m_proxyCount * 2) {
            break
          }
        }else {
          xIndex--;
          if(xIndex < 0) {
            break
          }
        }
        xProgress = (m_bounds[0][xIndex].value - p1x) / dx
      }else {
        if(yProgress > subInput.maxFraction) {
          break
        }
        if(sy > 0 ? m_bounds[1][yIndex].IsLower() : m_bounds[1][yIndex].IsUpper()) {
          proxy = m_bounds[1][yIndex].proxy;
          if(sx >= 0) {
            if(proxy.lowerBounds[0] <= xIndex - 1 && proxy.upperBounds[0] >= xIndex) {
              subInput.maxFraction = callback(proxy, subInput)
            }
          }else {
            if(proxy.lowerBounds[0] <= xIndex && proxy.upperBounds[0] >= xIndex + 1) {
              subInput.maxFraction = callback(proxy, subInput)
            }
          }
        }
        if(subInput.maxFraction == 0) {
          break
        }
        if(sy > 0) {
          yIndex++;
          if(yIndex == m_proxyCount * 2) {
            break
          }
        }else {
          yIndex--;
          if(yIndex < 0) {
            break
          }
        }
        yProgress = (m_bounds[1][yIndex].value - p1y) / dy
      }
    }
    break
  }
  this.m_queryResultCount = 0;
  this.IncrementTimeStamp();
  return
};
b2BroadPhase.prototype.TestOverlapBound = function(b, p) {
  for(var axis = 0;axis < 2;++axis) {
    var bounds = m_bounds[axis];
    var bound = bounds[p.upperBounds[axis]];
    if(b.lowerValues[axis] > bound.value) {
      return false
    }
    bound = bounds[p.lowerBounds[axis]];
    if(b.upperValues[axis] < bound.value) {
      return false
    }
  }
  return true
};var b2WorldManifold = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2WorldManifold.prototype.__constructor = function() {
  this.m_points = new Array(b2Settings.b2_maxManifoldPoints);
  for(var i = 0;i < b2Settings.b2_maxManifoldPoints;i++) {
    this.m_points[i] = new b2Vec2
  }
};
b2WorldManifold.prototype.__varz = function() {
  this.m_normal = new b2Vec2
};
b2WorldManifold.prototype.m_normal = new b2Vec2;
b2WorldManifold.prototype.m_points = null;
b2WorldManifold.prototype.Initialize = function(manifold, xfA, radiusA, xfB, radiusB) {
  if(manifold.m_pointCount == 0) {
    return
  }
  var i = 0;
  var tVec;
  var tMat;
  var normalX;
  var normalY;
  var planePointX;
  var planePointY;
  var clipPointX;
  var clipPointY;
  switch(manifold.m_type) {
    case b2Manifold.e_circles:
      tMat = xfA.R;
      tVec = manifold.m_localPoint;
      var pointAX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      var pointAY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      tMat = xfB.R;
      tVec = manifold.m_points[0].m_localPoint;
      var pointBX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      var pointBY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      var dX = pointBX - pointAX;
      var dY = pointBY - pointAY;
      var d2 = dX * dX + dY * dY;
      if(d2 > Number.MIN_VALUE * Number.MIN_VALUE) {
        var d = Math.sqrt(d2);
        this.m_normal.x = dX / d;
        this.m_normal.y = dY / d
      }else {
        this.m_normal.x = 1;
        this.m_normal.y = 0
      }
      var cAX = pointAX + radiusA * this.m_normal.x;
      var cAY = pointAY + radiusA * this.m_normal.y;
      var cBX = pointBX - radiusB * this.m_normal.x;
      var cBY = pointBY - radiusB * this.m_normal.y;
      this.m_points[0].x = 0.5 * (cAX + cBX);
      this.m_points[0].y = 0.5 * (cAY + cBY);
      break;
    case b2Manifold.e_faceA:
      tMat = xfA.R;
      tVec = manifold.m_localPlaneNormal;
      normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      tMat = xfA.R;
      tVec = manifold.m_localPoint;
      planePointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      planePointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      this.m_normal.x = normalX;
      this.m_normal.y = normalY;
      for(i = 0;i < manifold.m_pointCount;i++) {
        tMat = xfB.R;
        tVec = manifold.m_points[i].m_localPoint;
        clipPointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        clipPointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        this.m_points[i].x = clipPointX + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalX;
        this.m_points[i].y = clipPointY + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalY
      }
      break;
    case b2Manifold.e_faceB:
      tMat = xfB.R;
      tVec = manifold.m_localPlaneNormal;
      normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      tMat = xfB.R;
      tVec = manifold.m_localPoint;
      planePointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      planePointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      this.m_normal.x = -normalX;
      this.m_normal.y = -normalY;
      for(i = 0;i < manifold.m_pointCount;i++) {
        tMat = xfA.R;
        tVec = manifold.m_points[i].m_localPoint;
        clipPointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        clipPointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        this.m_points[i].x = clipPointX + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalX;
        this.m_points[i].y = clipPointY + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalY
      }
      break
  }
};var b2RayCastOutput = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2RayCastOutput.prototype.__constructor = function() {
};
b2RayCastOutput.prototype.__varz = function() {
  this.normal = new b2Vec2
};
b2RayCastOutput.prototype.normal = new b2Vec2;
b2RayCastOutput.prototype.fraction = null;var b2ConstantForceController = function() {
  b2Controller.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2ConstantForceController.prototype, b2Controller.prototype);
b2ConstantForceController.prototype._super = function() {
  b2Controller.prototype.__constructor.apply(this, arguments)
};
b2ConstantForceController.prototype.__constructor = function() {
};
b2ConstantForceController.prototype.__varz = function() {
  this.F = new b2Vec2(0, 0)
};
b2ConstantForceController.prototype.F = new b2Vec2(0, 0);
b2ConstantForceController.prototype.Step = function(step) {
  for(var i = m_bodyList;i;i = i.nextBody) {
    var body = i.body;
    if(!body.IsAwake()) {
      continue
    }
    body.ApplyForce(this.F, body.GetWorldCenter())
  }
};var b2MassData = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2MassData.prototype.__constructor = function() {
};
b2MassData.prototype.__varz = function() {
  this.center = new b2Vec2(0, 0)
};
b2MassData.prototype.mass = 0;
b2MassData.prototype.center = new b2Vec2(0, 0);
b2MassData.prototype.I = 0;var b2Joint = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Joint.prototype.__constructor = function(def) {
  b2Settings.b2Assert(def.bodyA != def.bodyB);
  m_type = def.type;
  m_prev = null;
  m_next = null;
  m_bodyA = def.bodyA;
  m_bodyB = def.bodyB;
  m_collideConnected = def.collideConnected;
  m_islandFlag = false;
  this.m_userData = def.userData
};
b2Joint.prototype.__varz = function() {
};
b2Joint.prototype.m_userData = null;
b2Joint.prototype.GetType = function() {
  return m_type
};
b2Joint.prototype.GetAnchorA = function() {
  return null
};
b2Joint.prototype.GetAnchorB = function() {
  return null
};
b2Joint.prototype.GetReactionForce = function(inv_dt) {
  return null
};
b2Joint.prototype.GetReactionTorque = function(inv_dt) {
  return 0
};
b2Joint.prototype.GetBodyA = function() {
  return m_bodyA
};
b2Joint.prototype.GetBodyB = function() {
  return m_bodyB
};
b2Joint.prototype.GetNext = function() {
  return m_next
};
b2Joint.prototype.GetUserData = function() {
  return this.m_userData
};
b2Joint.prototype.SetUserData = function(data) {
  this.m_userData = data
};
b2Joint.prototype.IsActive = function() {
  return m_bodyA.IsActive() && m_bodyB.IsActive()
};var b2DynamicTree = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2DynamicTree.prototype.__constructor = function() {
  this.m_root = null;
  this.m_freeList = null;
  this.m_path = 0;
  this.m_insertionCount = 0
};
b2DynamicTree.prototype.__varz = function() {
};
b2DynamicTree.prototype.m_root = null;
b2DynamicTree.prototype.m_freeList = null;
b2DynamicTree.prototype.m_path = 0;
b2DynamicTree.prototype.m_insertionCount = 0;
b2DynamicTree.prototype.AllocateNode = function() {
  if(this.m_freeList) {
    var node = this.m_freeList;
    this.m_freeList = node.parent;
    node.parent = null;
    node.child1 = null;
    node.child2 = null;
    return node
  }
  return new b2DynamicTreeNode
};
b2DynamicTree.prototype.FreeNode = function(node) {
  node.parent = this.m_freeList;
  this.m_freeList = node
};
b2DynamicTree.prototype.InsertLeaf = function(leaf) {
  ++this.m_insertionCount;
  if(this.m_root == null) {
    this.m_root = leaf;
    this.m_root.parent = null;
    return
  }
  var center = leaf.aabb.GetCenter();
  var sibling = this.m_root;
  if(sibling.IsLeaf() == false) {
    do {
      var child1 = sibling.child1;
      var child2 = sibling.child2;
      var norm1 = Math.abs((child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 - center.x) + Math.abs((child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 - center.y);
      var norm2 = Math.abs((child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 - center.x) + Math.abs((child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 - center.y);
      if(norm1 < norm2) {
        sibling = child1
      }else {
        sibling = child2
      }
    }while(sibling.IsLeaf() == false)
  }
  var node1 = sibling.parent;
  var node2 = this.AllocateNode();
  node2.parent = node1;
  node2.userData = null;
  node2.aabb.Combine(leaf.aabb, sibling.aabb);
  if(node1) {
    if(sibling.parent.child1 == sibling) {
      node1.child1 = node2
    }else {
      node1.child2 = node2
    }
    node2.child1 = sibling;
    node2.child2 = leaf;
    sibling.parent = node2;
    leaf.parent = node2;
    do {
      if(node1.aabb.Contains(node2.aabb)) {
        break
      }
      node1.aabb.Combine(node1.child1.aabb, node1.child2.aabb);
      node2 = node1;
      node1 = node1.parent
    }while(node1)
  }else {
    node2.child1 = sibling;
    node2.child2 = leaf;
    sibling.parent = node2;
    leaf.parent = node2;
    this.m_root = node2
  }
};
b2DynamicTree.prototype.RemoveLeaf = function(leaf) {
  if(leaf == this.m_root) {
    this.m_root = null;
    return
  }
  var node2 = leaf.parent;
  var node1 = node2.parent;
  var sibling;
  if(node2.child1 == leaf) {
    sibling = node2.child2
  }else {
    sibling = node2.child1
  }
  if(node1) {
    if(node1.child1 == node2) {
      node1.child1 = sibling
    }else {
      node1.child2 = sibling
    }
    sibling.parent = node1;
    this.FreeNode(node2);
    while(node1) {
      var oldAABB = node1.aabb;
      node1.aabb = b2AABB.Combine(node1.child1.aabb, node1.child2.aabb);
      if(oldAABB.Contains(node1.aabb)) {
        break
      }
      node1 = node1.parent
    }
  }else {
    this.m_root = sibling;
    sibling.parent = null;
    this.FreeNode(node2)
  }
};
b2DynamicTree.prototype.CreateProxy = function(aabb, userData) {
  var node = this.AllocateNode();
  var extendX = b2Settings.b2_aabbExtension;
  var extendY = b2Settings.b2_aabbExtension;
  node.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
  node.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
  node.aabb.upperBound.x = aabb.upperBound.x + extendX;
  node.aabb.upperBound.y = aabb.upperBound.y + extendY;
  node.userData = userData;
  this.InsertLeaf(node);
  return node
};
b2DynamicTree.prototype.DestroyProxy = function(proxy) {
  this.RemoveLeaf(proxy);
  this.FreeNode(proxy)
};
b2DynamicTree.prototype.MoveProxy = function(proxy, aabb, displacement) {
  b2Settings.b2Assert(proxy.IsLeaf());
  if(proxy.aabb.Contains(aabb)) {
    return false
  }
  this.RemoveLeaf(proxy);
  var extendX = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.x > 0 ? displacement.x : -displacement.x);
  var extendY = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.y > 0 ? displacement.y : -displacement.y);
  proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
  proxy.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
  proxy.aabb.upperBound.x = aabb.upperBound.x + extendX;
  proxy.aabb.upperBound.y = aabb.upperBound.y + extendY;
  this.InsertLeaf(proxy);
  return true
};
b2DynamicTree.prototype.Rebalance = function(iterations) {
  if(this.m_root == null) {
    return
  }
  for(var i = 0;i < iterations;i++) {
    var node = this.m_root;
    var bit = 0;
    while(node.IsLeaf() == false) {
      node = this.m_path >> bit & 1 ? node.child2 : node.child1;
      bit = bit + 1 & 31
    }
    ++this.m_path;
    this.RemoveLeaf(node);
    this.InsertLeaf(node)
  }
};
b2DynamicTree.prototype.GetFatAABB = function(proxy) {
  return proxy.aabb
};
b2DynamicTree.prototype.GetUserData = function(proxy) {
  return proxy.userData
};
b2DynamicTree.prototype.Query = function(callback, aabb) {
  if(this.m_root == null) {
    return
  }
  var stack = new Array;
  var count = 0;
  stack[count++] = this.m_root;
  while(count > 0) {
    var node = stack[--count];
    if(node.aabb.TestOverlap(aabb)) {
      if(node.IsLeaf()) {
        var proceed = callback(node);
        if(!proceed) {
          return
        }
      }else {
        stack[count++] = node.child1;
        stack[count++] = node.child2
      }
    }
  }
};
b2DynamicTree.prototype.RayCast = function(callback, input) {
  if(this.m_root == null) {
    return
  }
  var p1 = input.p1;
  var p2 = input.p2;
  var r = b2Math.SubtractVV(p1, p2);
  r.Normalize();
  var v = b2Math.CrossFV(1, r);
  var abs_v = b2Math.AbsV(v);
  var maxFraction = input.maxFraction;
  var segmentAABB = new b2AABB;
  var tX;
  var tY;
  tX = p1.x + maxFraction * (p2.x - p1.x);
  tY = p1.y + maxFraction * (p2.y - p1.y);
  segmentAABB.lowerBound.x = Math.min(p1.x, tX);
  segmentAABB.lowerBound.y = Math.min(p1.y, tY);
  segmentAABB.upperBound.x = Math.max(p1.x, tX);
  segmentAABB.upperBound.y = Math.max(p1.y, tY);
  var stack = new Array;
  var count = 0;
  stack[count++] = this.m_root;
  while(count > 0) {
    var node = stack[--count];
    if(node.aabb.TestOverlap(segmentAABB) == false) {
      continue
    }
    var c = node.aabb.GetCenter();
    var h = node.aabb.GetExtents();
    var separation = Math.abs(v.x * (p1.x - c.x) + v.y * (p1.y - c.y)) - abs_v.x * h.x - abs_v.y * h.y;
    if(separation > 0) {
      continue
    }
    if(node.IsLeaf()) {
      var subInput = new b2RayCastInput;
      subInput.p1 = input.p1;
      subInput.p2 = input.p2;
      subInput.maxFraction = input.maxFraction;
      maxFraction = callback(subInput, node);
      if(maxFraction == 0) {
        return
      }
      tX = p1.x + maxFraction * (p2.x - p1.x);
      tY = p1.y + maxFraction * (p2.y - p1.y);
      segmentAABB.lowerBound.x = Math.min(p1.x, tX);
      segmentAABB.lowerBound.y = Math.min(p1.y, tY);
      segmentAABB.upperBound.x = Math.max(p1.x, tX);
      segmentAABB.upperBound.y = Math.max(p1.y, tY)
    }else {
      stack[count++] = node.child1;
      stack[count++] = node.child2
    }
  }
};var b2JointEdge = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2JointEdge.prototype.__constructor = function() {
};
b2JointEdge.prototype.__varz = function() {
};
b2JointEdge.prototype.other = null;
b2JointEdge.prototype.joint = null;
b2JointEdge.prototype.prev = null;
b2JointEdge.prototype.next = null;var b2RayCastInput = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2RayCastInput.prototype.__constructor = function() {
};
b2RayCastInput.prototype.__varz = function() {
  this.p1 = new b2Vec2;
  this.p2 = new b2Vec2
};
b2RayCastInput.prototype.p1 = new b2Vec2;
b2RayCastInput.prototype.p2 = new b2Vec2;
b2RayCastInput.prototype.maxFraction = null;var Features = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
Features.prototype.__constructor = function() {
};
Features.prototype.__varz = function() {
};
Features.prototype._referenceEdge = 0;
Features.prototype._incidentEdge = 0;
Features.prototype._incidentVertex = 0;
Features.prototype._flip = 0;
Features.prototype._m_id = null;
Features.prototype.__defineGetter__("referenceEdge", function() {
  return this._referenceEdge
});
Features.prototype.__defineSetter__("referenceEdge", function(value) {
  this._referenceEdge = value;
  this._m_id._key = this._m_id._key & 4294967040 | this._referenceEdge & 255
});
Features.prototype.__defineGetter__("incidentEdge", function() {
  return this._incidentEdge
});
Features.prototype.__defineSetter__("incidentEdge", function(value) {
  this._incidentEdge = value;
  this._m_id._key = this._m_id._key & 4294902015 | this._incidentEdge << 8 & 65280
});
Features.prototype.__defineGetter__("incidentVertex", function() {
  return this._incidentVertex
});
Features.prototype.__defineSetter__("incidentVertex", function(value) {
  this._incidentVertex = value;
  this._m_id._key = this._m_id._key & 4278255615 | this._incidentVertex << 16 & 16711680
});
Features.prototype.__defineGetter__("flip", function() {
  return this._flip
});
Features.prototype.__defineSetter__("flip", function(value) {
  this._flip = value;
  this._m_id._key = this._m_id._key & 16777215 | this._flip << 24 & 4278190080
});var b2FilterData = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2FilterData.prototype.__constructor = function() {
};
b2FilterData.prototype.__varz = function() {
  this.categoryBits = 1;
  this.maskBits = 65535
};
b2FilterData.prototype.categoryBits = 1;
b2FilterData.prototype.maskBits = 65535;
b2FilterData.prototype.groupIndex = 0;
b2FilterData.prototype.Copy = function() {
  var copy = new b2FilterData;
  copy.categoryBits = this.categoryBits;
  copy.maskBits = this.maskBits;
  copy.groupIndex = this.groupIndex;
  return copy
};var b2AABB = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2AABB.prototype.__constructor = function() {
};
b2AABB.prototype.__varz = function() {
  this.lowerBound = new b2Vec2;
  this.upperBound = new b2Vec2
};
b2AABB.Combine = function(aabb1, aabb2) {
  var aabb = new b2AABB;
  aabb.Combine(aabb1, aabb2);
  return aabb
};
b2AABB.prototype.lowerBound = new b2Vec2;
b2AABB.prototype.upperBound = new b2Vec2;
b2AABB.prototype.IsValid = function() {
  var dX = this.upperBound.x - this.lowerBound.x;
  var dY = this.upperBound.y - this.lowerBound.y;
  var valid = dX >= 0 && dY >= 0;
  valid = valid && this.lowerBound.IsValid() && this.upperBound.IsValid();
  return valid
};
b2AABB.prototype.GetCenter = function() {
  return new b2Vec2((this.lowerBound.x + this.upperBound.x) / 2, (this.lowerBound.y + this.upperBound.y) / 2)
};
b2AABB.prototype.GetExtents = function() {
  return new b2Vec2((this.upperBound.x - this.lowerBound.x) / 2, (this.upperBound.y - this.lowerBound.y) / 2)
};
b2AABB.prototype.Contains = function(aabb) {
  var result = true && this.lowerBound.x <= aabb.lowerBound.x && this.lowerBound.y <= aabb.lowerBound.y && aabb.upperBound.x <= this.upperBound.x && aabb.upperBound.y <= this.upperBound.y;
  return result
};
b2AABB.prototype.RayCast = function(output, input) {
  var tmin = -Number.MAX_VALUE;
  var tmax = Number.MAX_VALUE;
  var pX = input.p1.x;
  var pY = input.p1.y;
  var dX = input.p2.x - input.p1.x;
  var dY = input.p2.y - input.p1.y;
  var absDX = Math.abs(dX);
  var absDY = Math.abs(dY);
  var normal = output.normal;
  var inv_d;
  var t1;
  var t2;
  var t3;
  var s;
  if(absDX < Number.MIN_VALUE) {
    if(pX < this.lowerBound.x || this.upperBound.x < pX) {
      return false
    }
  }else {
    inv_d = 1 / dX;
    t1 = (this.lowerBound.x - pX) * inv_d;
    t2 = (this.upperBound.x - pX) * inv_d;
    s = -1;
    if(t1 > t2) {
      t3 = t1;
      t1 = t2;
      t2 = t3;
      s = 1
    }
    if(t1 > tmin) {
      normal.x = s;
      normal.y = 0;
      tmin = t1
    }
    tmax = Math.min(tmax, t2);
    if(tmin > tmax) {
      return false
    }
  }
  if(absDY < Number.MIN_VALUE) {
    if(pY < this.lowerBound.y || this.upperBound.y < pY) {
      return false
    }
  }else {
    inv_d = 1 / dY;
    t1 = (this.lowerBound.y - pY) * inv_d;
    t2 = (this.upperBound.y - pY) * inv_d;
    s = -1;
    if(t1 > t2) {
      t3 = t1;
      t1 = t2;
      t2 = t3;
      s = 1
    }
    if(t1 > tmin) {
      normal.y = s;
      normal.x = 0;
      tmin = t1
    }
    tmax = Math.min(tmax, t2);
    if(tmin > tmax) {
      return false
    }
  }
  output.fraction = tmin;
  return true
};
b2AABB.prototype.TestOverlap = function(other) {
  var d1X = other.lowerBound.x - this.upperBound.x;
  var d1Y = other.lowerBound.y - this.upperBound.y;
  var d2X = this.lowerBound.x - other.upperBound.x;
  var d2Y = this.lowerBound.y - other.upperBound.y;
  if(d1X > 0 || d1Y > 0) {
    return false
  }
  if(d2X > 0 || d2Y > 0) {
    return false
  }
  return true
};
b2AABB.prototype.Combine = function(aabb1, aabb2) {
  this.lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x);
  this.lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y);
  this.upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x);
  this.upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y)
};var b2Jacobian = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Jacobian.prototype.__constructor = function() {
};
b2Jacobian.prototype.__varz = function() {
  this.linearA = new b2Vec2;
  this.linearB = new b2Vec2
};
b2Jacobian.prototype.linearA = new b2Vec2;
b2Jacobian.prototype.angularA = null;
b2Jacobian.prototype.linearB = new b2Vec2;
b2Jacobian.prototype.angularB = null;
b2Jacobian.prototype.SetZero = function() {
  this.linearA.SetZero();
  this.angularA = 0;
  this.linearB.SetZero();
  this.angularB = 0
};
b2Jacobian.prototype.Set = function(x1, a1, x2, a2) {
  this.linearA.SetV(x1);
  this.angularA = a1;
  this.linearB.SetV(x2);
  this.angularB = a2
};
b2Jacobian.prototype.Compute = function(x1, a1, x2, a2) {
  return this.linearA.x * x1.x + this.linearA.y * x1.y + this.angularA * a1 + (this.linearB.x * x2.x + this.linearB.y * x2.y) + this.angularB * a2
};var b2Bound = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Bound.prototype.__constructor = function() {
};
b2Bound.prototype.__varz = function() {
};
b2Bound.prototype.value = 0;
b2Bound.prototype.proxy = null;
b2Bound.prototype.stabbingCount = 0;
b2Bound.prototype.IsLower = function() {
  return(this.value & 1) == 0
};
b2Bound.prototype.IsUpper = function() {
  return(this.value & 1) == 1
};
b2Bound.prototype.Swap = function(b) {
  var tempValue = this.value;
  var tempProxy = this.proxy;
  var tempStabbingCount = this.stabbingCount;
  this.value = b.value;
  this.proxy = b.proxy;
  this.stabbingCount = b.stabbingCount;
  b.value = tempValue;
  b.proxy = tempProxy;
  b.stabbingCount = tempStabbingCount
};var b2SimplexVertex = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2SimplexVertex.prototype.__constructor = function() {
};
b2SimplexVertex.prototype.__varz = function() {
};
b2SimplexVertex.prototype.wA = null;
b2SimplexVertex.prototype.wB = null;
b2SimplexVertex.prototype.w = null;
b2SimplexVertex.prototype.a = null;
b2SimplexVertex.prototype.indexA = 0;
b2SimplexVertex.prototype.indexB = 0;
b2SimplexVertex.prototype.Set = function(other) {
  this.wA.SetV(other.wA);
  this.wB.SetV(other.wB);
  this.w.SetV(other.w);
  this.a = other.a;
  this.indexA = other.indexA;
  this.indexB = other.indexB
};var b2Collision = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Collision.prototype.__constructor = function() {
};
b2Collision.prototype.__varz = function() {
};
b2Collision.b2_nullFeature = 255;
b2Collision.s_incidentEdge = b2Collision.MakeClipPointVector();
b2Collision.s_clipPoints1 = b2Collision.MakeClipPointVector();
b2Collision.s_clipPoints2 = b2Collision.MakeClipPointVector();
b2Collision.s_edgeAO = new Array(1);
b2Collision.s_edgeBO = new Array(1);
b2Collision.s_localTangent = new b2Vec2;
b2Collision.s_localNormal = new b2Vec2;
b2Collision.s_planePoint = new b2Vec2;
b2Collision.s_normal = new b2Vec2;
b2Collision.s_tangent = new b2Vec2;
b2Collision.s_tangent2 = new b2Vec2;
b2Collision.s_v11 = new b2Vec2;
b2Collision.s_v12 = new b2Vec2;
b2Collision.b2CollidePolyTempVec = new b2Vec2;
b2Collision.MakeClipPointVector = function() {
  var r = new Array(2);
  r[0] = new ClipVertex;
  r[1] = new ClipVertex;
  return r
};
b2Collision.ClipSegmentToLine = function(vOut, vIn, normal, offset) {
  var cv;
  var numOut = 0;
  cv = vIn[0];
  var vIn0 = cv.v;
  cv = vIn[1];
  var vIn1 = cv.v;
  var distance0 = normal.x * vIn0.x + normal.y * vIn0.y - offset;
  var distance1 = normal.x * vIn1.x + normal.y * vIn1.y - offset;
  if(distance0 <= 0) {
    vOut[numOut++].Set(vIn[0])
  }
  if(distance1 <= 0) {
    vOut[numOut++].Set(vIn[1])
  }
  if(distance0 * distance1 < 0) {
    var interp = distance0 / (distance0 - distance1);
    cv = vOut[numOut];
    var tVec = cv.v;
    tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x);
    tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y);
    cv = vOut[numOut];
    var cv2;
    if(distance0 > 0) {
      cv2 = vIn[0];
      cv.id = cv2.id
    }else {
      cv2 = vIn[1];
      cv.id = cv2.id
    }
    ++numOut
  }
  return numOut
};
b2Collision.EdgeSeparation = function(poly1, xf1, edge1, poly2, xf2) {
  var count1 = poly1.m_vertexCount;
  var vertices1 = poly1.m_vertices;
  var normals1 = poly1.m_normals;
  var count2 = poly2.m_vertexCount;
  var vertices2 = poly2.m_vertices;
  var tMat;
  var tVec;
  tMat = xf1.R;
  tVec = normals1[edge1];
  var normal1WorldX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
  var normal1WorldY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
  tMat = xf2.R;
  var normal1X = tMat.col1.x * normal1WorldX + tMat.col1.y * normal1WorldY;
  var normal1Y = tMat.col2.x * normal1WorldX + tMat.col2.y * normal1WorldY;
  var index = 0;
  var minDot = Number.MAX_VALUE;
  for(var i = 0;i < count2;++i) {
    tVec = vertices2[i];
    var dot = tVec.x * normal1X + tVec.y * normal1Y;
    if(dot < minDot) {
      minDot = dot;
      index = i
    }
  }
  tVec = vertices1[edge1];
  tMat = xf1.R;
  var v1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
  var v1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
  tVec = vertices2[index];
  tMat = xf2.R;
  var v2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
  var v2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
  v2X -= v1X;
  v2Y -= v1Y;
  var separation = v2X * normal1WorldX + v2Y * normal1WorldY;
  return separation
};
b2Collision.FindMaxSeparation = function(edgeIndex, poly1, xf1, poly2, xf2) {
  var count1 = poly1.m_vertexCount;
  var normals1 = poly1.m_normals;
  var tVec;
  var tMat;
  tMat = xf2.R;
  tVec = poly2.m_centroid;
  var dX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
  var dY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
  tMat = xf1.R;
  tVec = poly1.m_centroid;
  dX -= xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
  dY -= xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
  var dLocal1X = dX * xf1.R.col1.x + dY * xf1.R.col1.y;
  var dLocal1Y = dX * xf1.R.col2.x + dY * xf1.R.col2.y;
  var edge = 0;
  var maxDot = -Number.MAX_VALUE;
  for(var i = 0;i < count1;++i) {
    tVec = normals1[i];
    var dot = tVec.x * dLocal1X + tVec.y * dLocal1Y;
    if(dot > maxDot) {
      maxDot = dot;
      edge = i
    }
  }
  var s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
  var prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
  var sPrev = b2Collision.EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
  var nextEdge = edge + 1 < count1 ? edge + 1 : 0;
  var sNext = b2Collision.EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
  var bestEdge = 0;
  var bestSeparation;
  var increment = 0;
  if(sPrev > s && sPrev > sNext) {
    increment = -1;
    bestEdge = prevEdge;
    bestSeparation = sPrev
  }else {
    if(sNext > s) {
      increment = 1;
      bestEdge = nextEdge;
      bestSeparation = sNext
    }else {
      edgeIndex[0] = edge;
      return s
    }
  }
  while(true) {
    if(increment == -1) {
      edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1
    }else {
      edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0
    }
    s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
    if(s > bestSeparation) {
      bestEdge = edge;
      bestSeparation = s
    }else {
      break
    }
  }
  edgeIndex[0] = bestEdge;
  return bestSeparation
};
b2Collision.FindIncidentEdge = function(c, poly1, xf1, edge1, poly2, xf2) {
  var count1 = poly1.m_vertexCount;
  var normals1 = poly1.m_normals;
  var count2 = poly2.m_vertexCount;
  var vertices2 = poly2.m_vertices;
  var normals2 = poly2.m_normals;
  var tMat;
  var tVec;
  tMat = xf1.R;
  tVec = normals1[edge1];
  var normal1X = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
  var normal1Y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
  tMat = xf2.R;
  var tX = tMat.col1.x * normal1X + tMat.col1.y * normal1Y;
  normal1Y = tMat.col2.x * normal1X + tMat.col2.y * normal1Y;
  normal1X = tX;
  var index = 0;
  var minDot = Number.MAX_VALUE;
  for(var i = 0;i < count2;++i) {
    tVec = normals2[i];
    var dot = normal1X * tVec.x + normal1Y * tVec.y;
    if(dot < minDot) {
      minDot = dot;
      index = i
    }
  }
  var tClip;
  var i1 = index;
  var i2 = i1 + 1 < count2 ? i1 + 1 : 0;
  tClip = c[0];
  tVec = vertices2[i1];
  tMat = xf2.R;
  tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
  tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
  tClip.id.features.referenceEdge = edge1;
  tClip.id.features.incidentEdge = i1;
  tClip.id.features.incidentVertex = 0;
  tClip = c[1];
  tVec = vertices2[i2];
  tMat = xf2.R;
  tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
  tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
  tClip.id.features.referenceEdge = edge1;
  tClip.id.features.incidentEdge = i2;
  tClip.id.features.incidentVertex = 1
};
b2Collision.CollidePolygons = function(manifold, polyA, xfA, polyB, xfB) {
  var cv;
  manifold.m_pointCount = 0;
  var totalRadius = polyA.m_radius + polyB.m_radius;
  var edgeA = 0;
  b2Collision.s_edgeAO[0] = edgeA;
  var separationA = b2Collision.FindMaxSeparation(b2Collision.s_edgeAO, polyA, xfA, polyB, xfB);
  edgeA = b2Collision.s_edgeAO[0];
  if(separationA > totalRadius) {
    return
  }
  var edgeB = 0;
  b2Collision.s_edgeBO[0] = edgeB;
  var separationB = b2Collision.FindMaxSeparation(b2Collision.s_edgeBO, polyB, xfB, polyA, xfA);
  edgeB = b2Collision.s_edgeBO[0];
  if(separationB > totalRadius) {
    return
  }
  var poly1;
  var poly2;
  var xf1;
  var xf2;
  var edge1 = 0;
  var flip = 0;
  var k_relativeTol = 0.98;
  var k_absoluteTol = 0.0010;
  var tMat;
  if(separationB > k_relativeTol * separationA + k_absoluteTol) {
    poly1 = polyB;
    poly2 = polyA;
    xf1 = xfB;
    xf2 = xfA;
    edge1 = edgeB;
    manifold.m_type = b2Manifold.e_faceB;
    flip = 1
  }else {
    poly1 = polyA;
    poly2 = polyB;
    xf1 = xfA;
    xf2 = xfB;
    edge1 = edgeA;
    manifold.m_type = b2Manifold.e_faceA;
    flip = 0
  }
  var incidentEdge = b2Collision.s_incidentEdge;
  b2Collision.FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);
  var count1 = poly1.m_vertexCount;
  var vertices1 = poly1.m_vertices;
  var local_v11 = vertices1[edge1];
  var local_v12;
  if(edge1 + 1 < count1) {
    local_v12 = vertices1[parseInt(edge1 + 1)]
  }else {
    local_v12 = vertices1[0]
  }
  var localTangent = b2Collision.s_localTangent;
  localTangent.Set(local_v12.x - local_v11.x, local_v12.y - local_v11.y);
  localTangent.Normalize();
  var localNormal = b2Collision.s_localNormal;
  localNormal.x = localTangent.y;
  localNormal.y = -localTangent.x;
  var planePoint = b2Collision.s_planePoint;
  planePoint.Set(0.5 * (local_v11.x + local_v12.x), 0.5 * (local_v11.y + local_v12.y));
  var tangent = b2Collision.s_tangent;
  tMat = xf1.R;
  tangent.x = tMat.col1.x * localTangent.x + tMat.col2.x * localTangent.y;
  tangent.y = tMat.col1.y * localTangent.x + tMat.col2.y * localTangent.y;
  var tangent2 = b2Collision.s_tangent2;
  tangent2.x = -tangent.x;
  tangent2.y = -tangent.y;
  var normal = b2Collision.s_normal;
  normal.x = tangent.y;
  normal.y = -tangent.x;
  var v11 = b2Collision.s_v11;
  var v12 = b2Collision.s_v12;
  v11.x = xf1.position.x + (tMat.col1.x * local_v11.x + tMat.col2.x * local_v11.y);
  v11.y = xf1.position.y + (tMat.col1.y * local_v11.x + tMat.col2.y * local_v11.y);
  v12.x = xf1.position.x + (tMat.col1.x * local_v12.x + tMat.col2.x * local_v12.y);
  v12.y = xf1.position.y + (tMat.col1.y * local_v12.x + tMat.col2.y * local_v12.y);
  var frontOffset = normal.x * v11.x + normal.y * v11.y;
  var sideOffset1 = -tangent.x * v11.x - tangent.y * v11.y + totalRadius;
  var sideOffset2 = tangent.x * v12.x + tangent.y * v12.y + totalRadius;
  var clipPoints1 = b2Collision.s_clipPoints1;
  var clipPoints2 = b2Collision.s_clipPoints2;
  var np = 0;
  np = b2Collision.ClipSegmentToLine(clipPoints1, incidentEdge, tangent2, sideOffset1);
  if(np < 2) {
    return
  }
  np = b2Collision.ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2);
  if(np < 2) {
    return
  }
  manifold.m_localPlaneNormal.SetV(localNormal);
  manifold.m_localPoint.SetV(planePoint);
  var pointCount = 0;
  for(var i = 0;i < b2Settings.b2_maxManifoldPoints;++i) {
    cv = clipPoints2[i];
    var separation = normal.x * cv.v.x + normal.y * cv.v.y - frontOffset;
    if(separation <= totalRadius) {
      var cp = manifold.m_points[pointCount];
      tMat = xf2.R;
      var tX = cv.v.x - xf2.position.x;
      var tY = cv.v.y - xf2.position.y;
      cp.m_localPoint.x = tX * tMat.col1.x + tY * tMat.col1.y;
      cp.m_localPoint.y = tX * tMat.col2.x + tY * tMat.col2.y;
      cp.m_id.Set(cv.id);
      cp.m_id.features.flip = flip;
      ++pointCount
    }
  }
  manifold.m_pointCount = pointCount
};
b2Collision.CollideCircles = function(manifold, circle1, xf1, circle2, xf2) {
  manifold.m_pointCount = 0;
  var tMat;
  var tVec;
  tMat = xf1.R;
  tVec = circle1.m_p;
  var p1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
  var p1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
  tMat = xf2.R;
  tVec = circle2.m_p;
  var p2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
  var p2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
  var dX = p2X - p1X;
  var dY = p2Y - p1Y;
  var distSqr = dX * dX + dY * dY;
  var radius = circle1.m_radius + circle2.m_radius;
  if(distSqr > radius * radius) {
    return
  }
  manifold.m_type = b2Manifold.e_circles;
  manifold.m_localPoint.SetV(circle1.m_p);
  manifold.m_localPlaneNormal.SetZero();
  manifold.m_pointCount = 1;
  manifold.m_points[0].m_localPoint.SetV(circle2.m_p);
  manifold.m_points[0].m_id.key = 0
};
b2Collision.CollidePolygonAndCircle = function(manifold, polygon, xf1, circle, xf2) {
  manifold.m_pointCount = 0;
  var tPoint;
  var dX;
  var dY;
  var positionX;
  var positionY;
  var tVec;
  var tMat;
  tMat = xf2.R;
  tVec = circle.m_p;
  var cX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
  var cY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
  dX = cX - xf1.position.x;
  dY = cY - xf1.position.y;
  tMat = xf1.R;
  var cLocalX = dX * tMat.col1.x + dY * tMat.col1.y;
  var cLocalY = dX * tMat.col2.x + dY * tMat.col2.y;
  var dist;
  var normalIndex = 0;
  var separation = -Number.MAX_VALUE;
  var radius = polygon.m_radius + circle.m_radius;
  var vertexCount = polygon.m_vertexCount;
  var vertices = polygon.m_vertices;
  var normals = polygon.m_normals;
  for(var i = 0;i < vertexCount;++i) {
    tVec = vertices[i];
    dX = cLocalX - tVec.x;
    dY = cLocalY - tVec.y;
    tVec = normals[i];
    var s = tVec.x * dX + tVec.y * dY;
    if(s > radius) {
      return
    }
    if(s > separation) {
      separation = s;
      normalIndex = i
    }
  }
  var vertIndex1 = normalIndex;
  var vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
  var v1 = vertices[vertIndex1];
  var v2 = vertices[vertIndex2];
  if(separation < Number.MIN_VALUE) {
    manifold.m_pointCount = 1;
    manifold.m_type = b2Manifold.e_faceA;
    manifold.m_localPlaneNormal.SetV(normals[normalIndex]);
    manifold.m_localPoint.x = 0.5 * (v1.x + v2.x);
    manifold.m_localPoint.y = 0.5 * (v1.y + v2.y);
    manifold.m_points[0].m_localPoint.SetV(circle.m_p);
    manifold.m_points[0].m_id.key = 0;
    return
  }
  var u1 = (cLocalX - v1.x) * (v2.x - v1.x) + (cLocalY - v1.y) * (v2.y - v1.y);
  var u2 = (cLocalX - v2.x) * (v1.x - v2.x) + (cLocalY - v2.y) * (v1.y - v2.y);
  if(u1 <= 0) {
    if((cLocalX - v1.x) * (cLocalX - v1.x) + (cLocalY - v1.y) * (cLocalY - v1.y) > radius * radius) {
      return
    }
    manifold.m_pointCount = 1;
    manifold.m_type = b2Manifold.e_faceA;
    manifold.m_localPlaneNormal.x = cLocalX - v1.x;
    manifold.m_localPlaneNormal.y = cLocalY - v1.y;
    manifold.m_localPlaneNormal.Normalize();
    manifold.m_localPoint.SetV(v1);
    manifold.m_points[0].m_localPoint.SetV(circle.m_p);
    manifold.m_points[0].m_id.key = 0
  }else {
    if(u2 <= 0) {
      if((cLocalX - v2.x) * (cLocalX - v2.x) + (cLocalY - v2.y) * (cLocalY - v2.y) > radius * radius) {
        return
      }
      manifold.m_pointCount = 1;
      manifold.m_type = b2Manifold.e_faceA;
      manifold.m_localPlaneNormal.x = cLocalX - v2.x;
      manifold.m_localPlaneNormal.y = cLocalY - v2.y;
      manifold.m_localPlaneNormal.Normalize();
      manifold.m_localPoint.SetV(v2);
      manifold.m_points[0].m_localPoint.SetV(circle.m_p);
      manifold.m_points[0].m_id.key = 0
    }else {
      var faceCenterX = 0.5 * (v1.x + v2.x);
      var faceCenterY = 0.5 * (v1.y + v2.y);
      separation = (cLocalX - faceCenterX) * normals[vertIndex1].x + (cLocalY - faceCenterY) * normals[vertIndex1].y;
      if(separation > radius) {
        return
      }
      manifold.m_pointCount = 1;
      manifold.m_type = b2Manifold.e_faceA;
      manifold.m_localPlaneNormal.x = normals[vertIndex1].x;
      manifold.m_localPlaneNormal.y = normals[vertIndex1].y;
      manifold.m_localPlaneNormal.Normalize();
      manifold.m_localPoint.Set(faceCenterX, faceCenterY);
      manifold.m_points[0].m_localPoint.SetV(circle.m_p);
      manifold.m_points[0].m_id.key = 0
    }
  }
};
b2Collision.TestOverlap = function(a, b) {
  var t1 = b.lowerBound;
  var t2 = a.upperBound;
  var d1X = t1.x - t2.x;
  var d1Y = t1.y - t2.y;
  t1 = a.lowerBound;
  t2 = b.upperBound;
  var d2X = t1.x - t2.x;
  var d2Y = t1.y - t2.y;
  if(d1X > 0 || d1Y > 0) {
    return false
  }
  if(d2X > 0 || d2Y > 0) {
    return false
  }
  return true
};var b2Mat22 = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Mat22.prototype.__constructor = function() {
  this.col1.x = this.col2.y = 1
};
b2Mat22.prototype.__varz = function() {
  this.col1 = new b2Vec2;
  this.col2 = new b2Vec2
};
b2Mat22.FromAngle = function(angle) {
  var mat = new b2Mat22;
  mat.Set(angle);
  return mat
};
b2Mat22.FromVV = function(c1, c2) {
  var mat = new b2Mat22;
  mat.SetVV(c1, c2);
  return mat
};
b2Mat22.prototype.col1 = new b2Vec2;
b2Mat22.prototype.col2 = new b2Vec2;
b2Mat22.prototype.Set = function(angle) {
  var c = Math.cos(angle);
  var s = Math.sin(angle);
  this.col1.x = c;
  this.col2.x = -s;
  this.col1.y = s;
  this.col2.y = c
};
b2Mat22.prototype.SetVV = function(c1, c2) {
  this.col1.SetV(c1);
  this.col2.SetV(c2)
};
b2Mat22.prototype.Copy = function() {
  var mat = new b2Mat22;
  mat.SetM(this);
  return mat
};
b2Mat22.prototype.SetM = function(m) {
  this.col1.SetV(m.col1);
  this.col2.SetV(m.col2)
};
b2Mat22.prototype.AddM = function(m) {
  this.col1.x += m.col1.x;
  this.col1.y += m.col1.y;
  this.col2.x += m.col2.x;
  this.col2.y += m.col2.y
};
b2Mat22.prototype.SetIdentity = function() {
  this.col1.x = 1;
  this.col2.x = 0;
  this.col1.y = 0;
  this.col2.y = 1
};
b2Mat22.prototype.SetZero = function() {
  this.col1.x = 0;
  this.col2.x = 0;
  this.col1.y = 0;
  this.col2.y = 0
};
b2Mat22.prototype.GetAngle = function() {
  return Math.atan2(this.col1.y, this.col1.x)
};
b2Mat22.prototype.GetInverse = function(out) {
  var a = this.col1.x;
  var b = this.col2.x;
  var c = this.col1.y;
  var d = this.col2.y;
  var det = a * d - b * c;
  if(det != 0) {
    det = 1 / det
  }
  out.col1.x = det * d;
  out.col2.x = -det * b;
  out.col1.y = -det * c;
  out.col2.y = det * a;
  return out
};
b2Mat22.prototype.Solve = function(out, bX, bY) {
  var a11 = this.col1.x;
  var a12 = this.col2.x;
  var a21 = this.col1.y;
  var a22 = this.col2.y;
  var det = a11 * a22 - a12 * a21;
  if(det != 0) {
    det = 1 / det
  }
  out.x = det * (a22 * bX - a12 * bY);
  out.y = det * (a11 * bY - a21 * bX);
  return out
};
b2Mat22.prototype.Abs = function() {
  this.col1.Abs();
  this.col2.Abs()
};var b2SimplexCache = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2SimplexCache.prototype.__constructor = function() {
};
b2SimplexCache.prototype.__varz = function() {
  this.indexA = new Array(3);
  this.indexB = new Array(3)
};
b2SimplexCache.prototype.metric = null;
b2SimplexCache.prototype.count = 0;
b2SimplexCache.prototype.indexA = new Array(3);
b2SimplexCache.prototype.indexB = new Array(3);var b2Shape = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Shape.prototype.__constructor = function() {
  m_type = e_unknownShape;
  m_radius = b2Settings.b2_linearSlop
};
b2Shape.prototype.__varz = function() {
};
b2Shape.e_hitCollide = 1;
b2Shape.e_missCollide = 0;
b2Shape.e_startsInsideCollide = -1;
b2Shape.TestOverlap = function(shape1, transform1, shape2, transform2) {
  var input = new b2DistanceInput;
  input.proxyA = new b2DistanceProxy;
  input.proxyA.Set(shape1);
  input.proxyB = new b2DistanceProxy;
  input.proxyB.Set(shape2);
  input.transformA = transform1;
  input.transformB = transform2;
  input.useRadii = true;
  var simplexCache = new b2SimplexCache;
  simplexCache.count = 0;
  var output = new b2DistanceOutput;
  b2Distance.Distance(output, simplexCache, input);
  return output.distance < 10 * Number.MIN_VALUE
};
b2Shape.prototype.Copy = function() {
  return null
};
b2Shape.prototype.Set = function(other) {
  m_radius = other.m_radius
};
b2Shape.prototype.GetType = function() {
  return m_type
};
b2Shape.prototype.TestPoint = function(xf, p) {
  return false
};
b2Shape.prototype.RayCast = function(output, input, transform) {
  return false
};
b2Shape.prototype.ComputeAABB = function(aabb, xf) {
};
b2Shape.prototype.ComputeMass = function(massData, density) {
};
b2Shape.prototype.ComputeSubmergedArea = function(normal, offset, xf, c) {
  return 0
};var b2Segment = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Segment.prototype.__constructor = function() {
};
b2Segment.prototype.__varz = function() {
  this.p1 = new b2Vec2;
  this.p2 = new b2Vec2
};
b2Segment.prototype.p1 = new b2Vec2;
b2Segment.prototype.p2 = new b2Vec2;
b2Segment.prototype.TestSegment = function(lambda, normal, segment, maxLambda) {
  var s = segment.p1;
  var rX = segment.p2.x - s.x;
  var rY = segment.p2.y - s.y;
  var dX = this.p2.x - this.p1.x;
  var dY = this.p2.y - this.p1.y;
  var nX = dY;
  var nY = -dX;
  var k_slop = 100 * Number.MIN_VALUE;
  var denom = -(rX * nX + rY * nY);
  if(denom > k_slop) {
    var bX = s.x - this.p1.x;
    var bY = s.y - this.p1.y;
    var a = bX * nX + bY * nY;
    if(0 <= a && a <= maxLambda * denom) {
      var mu2 = -rX * bY + rY * bX;
      if(-k_slop * denom <= mu2 && mu2 <= denom * (1 + k_slop)) {
        a /= denom;
        var nLen = Math.sqrt(nX * nX + nY * nY);
        nX /= nLen;
        nY /= nLen;
        lambda[0] = a;
        normal.Set(nX, nY);
        return true
      }
    }
  }
  return false
};
b2Segment.prototype.Extend = function(aabb) {
  this.ExtendForward(aabb);
  this.ExtendBackward(aabb)
};
b2Segment.prototype.ExtendForward = function(aabb) {
  var dX = this.p2.x - this.p1.x;
  var dY = this.p2.y - this.p1.y;
  var lambda = Math.min(dX > 0 ? (aabb.upperBound.x - this.p1.x) / dX : dX < 0 ? (aabb.lowerBound.x - this.p1.x) / dX : Number.POSITIVE_INFINITY, dY > 0 ? (aabb.upperBound.y - this.p1.y) / dY : dY < 0 ? (aabb.lowerBound.y - this.p1.y) / dY : Number.POSITIVE_INFINITY);
  this.p2.x = this.p1.x + dX * lambda;
  this.p2.y = this.p1.y + dY * lambda
};
b2Segment.prototype.ExtendBackward = function(aabb) {
  var dX = -this.p2.x + this.p1.x;
  var dY = -this.p2.y + this.p1.y;
  var lambda = Math.min(dX > 0 ? (aabb.upperBound.x - this.p2.x) / dX : dX < 0 ? (aabb.lowerBound.x - this.p2.x) / dX : Number.POSITIVE_INFINITY, dY > 0 ? (aabb.upperBound.y - this.p2.y) / dY : dY < 0 ? (aabb.lowerBound.y - this.p2.y) / dY : Number.POSITIVE_INFINITY);
  this.p1.x = this.p2.x + dX * lambda;
  this.p1.y = this.p2.y + dY * lambda
};var b2ContactRegister = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ContactRegister.prototype.__constructor = function() {
};
b2ContactRegister.prototype.__varz = function() {
};
b2ContactRegister.prototype.createFcn = null;
b2ContactRegister.prototype.destroyFcn = null;
b2ContactRegister.prototype.primary = null;
b2ContactRegister.prototype.pool = null;
b2ContactRegister.prototype.poolCount = 0;var b2DebugDraw = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2DebugDraw.prototype.__constructor = function() {
  this.m_drawFlags = 0
};
b2DebugDraw.prototype.__varz = function() {
};
b2DebugDraw.e_shapeBit = 1;
b2DebugDraw.e_jointBit = 2;
b2DebugDraw.e_aabbBit = 4;
b2DebugDraw.e_pairBit = 8;
b2DebugDraw.e_centerOfMassBit = 16;
b2DebugDraw.e_controllerBit = 32;
b2DebugDraw.prototype.m_drawFlags = 0;
b2DebugDraw.prototype.m_drawScale = 1;
b2DebugDraw.prototype.m_lineThickness = 1;
b2DebugDraw.prototype.m_alpha = 1;
b2DebugDraw.prototype.m_fillAlpha = 1;
b2DebugDraw.prototype.m_xformScale = 1;
b2DebugDraw.prototype.SetFlags = function(flags) {
  this.m_drawFlags = flags
};
b2DebugDraw.prototype.GetFlags = function() {
  return this.m_drawFlags
};
b2DebugDraw.prototype.AppendFlags = function(flags) {
  this.m_drawFlags |= flags
};
b2DebugDraw.prototype.ClearFlags = function(flags) {
  this.m_drawFlags &= ~flags
};
b2DebugDraw.prototype.SetSprite = function(sprite) {
  m_sprite = sprite
};
b2DebugDraw.prototype.GetSprite = function() {
  return m_sprite
};
b2DebugDraw.prototype.SetDrawScale = function(drawScale) {
  this.m_drawScale = drawScale
};
b2DebugDraw.prototype.GetDrawScale = function() {
  return this.m_drawScale
};
b2DebugDraw.prototype.SetLineThickness = function(lineThickness) {
  this.m_lineThickness = lineThickness
};
b2DebugDraw.prototype.GetLineThickness = function() {
  return this.m_lineThickness
};
b2DebugDraw.prototype.SetAlpha = function(alpha) {
  this.m_alpha = alpha
};
b2DebugDraw.prototype.GetAlpha = function() {
  return this.m_alpha
};
b2DebugDraw.prototype.SetFillAlpha = function(alpha) {
  this.m_fillAlpha = alpha
};
b2DebugDraw.prototype.GetFillAlpha = function() {
  return this.m_fillAlpha
};
b2DebugDraw.prototype.SetXFormScale = function(xformScale) {
  this.m_xformScale = xformScale
};
b2DebugDraw.prototype.GetXFormScale = function() {
  return this.m_xformScale
};
b2DebugDraw.prototype.DrawPolygon = function(vertices, vertexCount, color) {
  m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
  m_sprite.graphics.moveTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
  for(var i = 1;i < vertexCount;i++) {
    m_sprite.graphics.lineTo(vertices[i].x * this.m_drawScale, vertices[i].y * this.m_drawScale)
  }
  m_sprite.graphics.lineTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale)
};
b2DebugDraw.prototype.DrawSolidPolygon = function(vertices, vertexCount, color) {
  m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
  m_sprite.graphics.moveTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
  m_sprite.graphics.beginFill(color.color, this.m_fillAlpha);
  for(var i = 1;i < vertexCount;i++) {
    m_sprite.graphics.lineTo(vertices[i].x * this.m_drawScale, vertices[i].y * this.m_drawScale)
  }
  m_sprite.graphics.lineTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
  m_sprite.graphics.endFill()
};
b2DebugDraw.prototype.DrawCircle = function(center, radius, color) {
  m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
  m_sprite.graphics.drawCircle(center.x * this.m_drawScale, center.y * this.m_drawScale, radius * this.m_drawScale)
};
b2DebugDraw.prototype.DrawSolidCircle = function(center, radius, axis, color) {
  m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
  m_sprite.graphics.moveTo(0, 0);
  m_sprite.graphics.beginFill(color.color, this.m_fillAlpha);
  m_sprite.graphics.drawCircle(center.x * this.m_drawScale, center.y * this.m_drawScale, radius * this.m_drawScale);
  m_sprite.graphics.endFill();
  m_sprite.graphics.moveTo(center.x * this.m_drawScale, center.y * this.m_drawScale);
  m_sprite.graphics.lineTo((center.x + axis.x * radius) * this.m_drawScale, (center.y + axis.y * radius) * this.m_drawScale)
};
b2DebugDraw.prototype.DrawSegment = function(p1, p2, color) {
  m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
  m_sprite.graphics.moveTo(p1.x * this.m_drawScale, p1.y * this.m_drawScale);
  m_sprite.graphics.lineTo(p2.x * this.m_drawScale, p2.y * this.m_drawScale)
};
b2DebugDraw.prototype.DrawTransform = function(xf) {
  m_sprite.graphics.lineStyle(this.m_lineThickness, 16711680, this.m_alpha);
  m_sprite.graphics.moveTo(xf.position.x * this.m_drawScale, xf.position.y * this.m_drawScale);
  m_sprite.graphics.lineTo((xf.position.x + this.m_xformScale * xf.R.col1.x) * this.m_drawScale, (xf.position.y + this.m_xformScale * xf.R.col1.y) * this.m_drawScale);
  m_sprite.graphics.lineStyle(this.m_lineThickness, 65280, this.m_alpha);
  m_sprite.graphics.moveTo(xf.position.x * this.m_drawScale, xf.position.y * this.m_drawScale);
  m_sprite.graphics.lineTo((xf.position.x + this.m_xformScale * xf.R.col2.x) * this.m_drawScale, (xf.position.y + this.m_xformScale * xf.R.col2.y) * this.m_drawScale)
};var b2Sweep = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Sweep.prototype.__constructor = function() {
};
b2Sweep.prototype.__varz = function() {
  this.localCenter = new b2Vec2;
  this.c0 = new b2Vec2;
  this.c = new b2Vec2
};
b2Sweep.prototype.localCenter = new b2Vec2;
b2Sweep.prototype.c0 = new b2Vec2;
b2Sweep.prototype.c = new b2Vec2;
b2Sweep.prototype.a0 = null;
b2Sweep.prototype.a = null;
b2Sweep.prototype.t0 = null;
b2Sweep.prototype.Set = function(other) {
  this.localCenter.SetV(other.localCenter);
  this.c0.SetV(other.c0);
  this.c.SetV(other.c);
  this.a0 = other.a0;
  this.a = other.a;
  this.t0 = other.t0
};
b2Sweep.prototype.Copy = function() {
  var copy = new b2Sweep;
  copy.localCenter.SetV(this.localCenter);
  copy.c0.SetV(this.c0);
  copy.c.SetV(this.c);
  copy.a0 = this.a0;
  copy.a = this.a;
  copy.t0 = this.t0;
  return copy
};
b2Sweep.prototype.GetTransform = function(xf, alpha) {
  xf.position.x = (1 - alpha) * this.c0.x + alpha * this.c.x;
  xf.position.y = (1 - alpha) * this.c0.y + alpha * this.c.y;
  var angle = (1 - alpha) * this.a0 + alpha * this.a;
  xf.R.Set(angle);
  var tMat = xf.R;
  xf.position.x -= tMat.col1.x * this.localCenter.x + tMat.col2.x * this.localCenter.y;
  xf.position.y -= tMat.col1.y * this.localCenter.x + tMat.col2.y * this.localCenter.y
};
b2Sweep.prototype.Advance = function(t) {
  if(this.t0 < t && 1 - this.t0 > Number.MIN_VALUE) {
    var alpha = (t - this.t0) / (1 - this.t0);
    this.c0.x = (1 - alpha) * this.c0.x + alpha * this.c.x;
    this.c0.y = (1 - alpha) * this.c0.y + alpha * this.c.y;
    this.a0 = (1 - alpha) * this.a0 + alpha * this.a;
    this.t0 = t
  }
};var b2DistanceOutput = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2DistanceOutput.prototype.__constructor = function() {
};
b2DistanceOutput.prototype.__varz = function() {
  this.pointA = new b2Vec2;
  this.pointB = new b2Vec2
};
b2DistanceOutput.prototype.pointA = new b2Vec2;
b2DistanceOutput.prototype.pointB = new b2Vec2;
b2DistanceOutput.prototype.distance = null;
b2DistanceOutput.prototype.iterations = 0;var b2Mat33 = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Mat33.prototype.__constructor = function(c1, c2, c3) {
  if(!c1 && !c2 && !c3) {
    this.col1.SetZero();
    this.col2.SetZero();
    this.col3.SetZero()
  }else {
    this.col1.SetV(c1);
    this.col2.SetV(c2);
    this.col3.SetV(c3)
  }
};
b2Mat33.prototype.__varz = function() {
  this.col1 = new b2Vec3;
  this.col2 = new b2Vec3;
  this.col3 = new b2Vec3
};
b2Mat33.prototype.col1 = new b2Vec3;
b2Mat33.prototype.col2 = new b2Vec3;
b2Mat33.prototype.col3 = new b2Vec3;
b2Mat33.prototype.SetVVV = function(c1, c2, c3) {
  this.col1.SetV(c1);
  this.col2.SetV(c2);
  this.col3.SetV(c3)
};
b2Mat33.prototype.Copy = function() {
  return new b2Mat33(this.col1, this.col2, this.col3)
};
b2Mat33.prototype.SetM = function(m) {
  this.col1.SetV(m.col1);
  this.col2.SetV(m.col2);
  this.col3.SetV(m.col3)
};
b2Mat33.prototype.AddM = function(m) {
  this.col1.x += m.col1.x;
  this.col1.y += m.col1.y;
  this.col1.z += m.col1.z;
  this.col2.x += m.col2.x;
  this.col2.y += m.col2.y;
  this.col2.z += m.col2.z;
  this.col3.x += m.col3.x;
  this.col3.y += m.col3.y;
  this.col3.z += m.col3.z
};
b2Mat33.prototype.SetIdentity = function() {
  this.col1.x = 1;
  this.col2.x = 0;
  this.col3.x = 0;
  this.col1.y = 0;
  this.col2.y = 1;
  this.col3.y = 0;
  this.col1.z = 0;
  this.col2.z = 0;
  this.col3.z = 1
};
b2Mat33.prototype.SetZero = function() {
  this.col1.x = 0;
  this.col2.x = 0;
  this.col3.x = 0;
  this.col1.y = 0;
  this.col2.y = 0;
  this.col3.y = 0;
  this.col1.z = 0;
  this.col2.z = 0;
  this.col3.z = 0
};
b2Mat33.prototype.Solve22 = function(out, bX, bY) {
  var a11 = this.col1.x;
  var a12 = this.col2.x;
  var a21 = this.col1.y;
  var a22 = this.col2.y;
  var det = a11 * a22 - a12 * a21;
  if(det != 0) {
    det = 1 / det
  }
  out.x = det * (a22 * bX - a12 * bY);
  out.y = det * (a11 * bY - a21 * bX);
  return out
};
b2Mat33.prototype.Solve33 = function(out, bX, bY, bZ) {
  var a11 = this.col1.x;
  var a21 = this.col1.y;
  var a31 = this.col1.z;
  var a12 = this.col2.x;
  var a22 = this.col2.y;
  var a32 = this.col2.z;
  var a13 = this.col3.x;
  var a23 = this.col3.y;
  var a33 = this.col3.z;
  var det = a11 * (a22 * a33 - a32 * a23) + a21 * (a32 * a13 - a12 * a33) + a31 * (a12 * a23 - a22 * a13);
  if(det != 0) {
    det = 1 / det
  }
  out.x = det * (bX * (a22 * a33 - a32 * a23) + bY * (a32 * a13 - a12 * a33) + bZ * (a12 * a23 - a22 * a13));
  out.y = det * (a11 * (bY * a33 - bZ * a23) + a21 * (bZ * a13 - bX * a33) + a31 * (bX * a23 - bY * a13));
  out.z = det * (a11 * (a22 * bZ - a32 * bY) + a21 * (a32 * bX - a12 * bZ) + a31 * (a12 * bY - a22 * bX));
  return out
};var b2PositionSolverManifold = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2PositionSolverManifold.prototype.__constructor = function() {
  this.m_normal = new b2Vec2;
  this.m_separations = new Array(b2Settings.b2_maxManifoldPoints);
  this.m_points = new Array(b2Settings.b2_maxManifoldPoints);
  for(var i = 0;i < b2Settings.b2_maxManifoldPoints;i++) {
    this.m_points[i] = new b2Vec2
  }
};
b2PositionSolverManifold.prototype.__varz = function() {
};
b2PositionSolverManifold.circlePointA = new b2Vec2;
b2PositionSolverManifold.circlePointB = new b2Vec2;
b2PositionSolverManifold.prototype.m_normal = null;
b2PositionSolverManifold.prototype.m_points = null;
b2PositionSolverManifold.prototype.m_separations = null;
b2PositionSolverManifold.prototype.Initialize = function(cc) {
  b2Settings.b2Assert(cc.pointCount > 0);
  var i = 0;
  var clipPointX;
  var clipPointY;
  var tMat;
  var tVec;
  var planePointX;
  var planePointY;
  switch(cc.type) {
    case b2Manifold.e_circles:
      tMat = cc.bodyA.m_xf.R;
      tVec = cc.localPoint;
      var pointAX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var pointAY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tMat = cc.bodyB.m_xf.R;
      tVec = cc.points[0].localPoint;
      var pointBX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var pointBY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      var dX = pointBX - pointAX;
      var dY = pointBY - pointAY;
      var d2 = dX * dX + dY * dY;
      if(d2 > Number.MIN_VALUE * Number.MIN_VALUE) {
        var d = Math.sqrt(d2);
        this.m_normal.x = dX / d;
        this.m_normal.y = dY / d
      }else {
        this.m_normal.x = 1;
        this.m_normal.y = 0
      }
      this.m_points[0].x = 0.5 * (pointAX + pointBX);
      this.m_points[0].y = 0.5 * (pointAY + pointBY);
      this.m_separations[0] = dX * this.m_normal.x + dY * this.m_normal.y - cc.radius;
      break;
    case b2Manifold.e_faceA:
      tMat = cc.bodyA.m_xf.R;
      tVec = cc.localPlaneNormal;
      this.m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      this.m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      tMat = cc.bodyA.m_xf.R;
      tVec = cc.localPoint;
      planePointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      planePointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tMat = cc.bodyB.m_xf.R;
      for(i = 0;i < cc.pointCount;++i) {
        tVec = cc.points[i].localPoint;
        clipPointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        clipPointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        this.m_separations[i] = (clipPointX - planePointX) * this.m_normal.x + (clipPointY - planePointY) * this.m_normal.y - cc.radius;
        this.m_points[i].x = clipPointX;
        this.m_points[i].y = clipPointY
      }
      break;
    case b2Manifold.e_faceB:
      tMat = cc.bodyB.m_xf.R;
      tVec = cc.localPlaneNormal;
      this.m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      this.m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      tMat = cc.bodyB.m_xf.R;
      tVec = cc.localPoint;
      planePointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      planePointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tMat = cc.bodyA.m_xf.R;
      for(i = 0;i < cc.pointCount;++i) {
        tVec = cc.points[i].localPoint;
        clipPointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        clipPointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        this.m_separations[i] = (clipPointX - planePointX) * this.m_normal.x + (clipPointY - planePointY) * this.m_normal.y - cc.radius;
        this.m_points[i].Set(clipPointX, clipPointY)
      }
      this.m_normal.x *= -1;
      this.m_normal.y *= -1;
      break
  }
};var b2OBB = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2OBB.prototype.__constructor = function() {
};
b2OBB.prototype.__varz = function() {
  this.R = new b2Mat22;
  this.center = new b2Vec2;
  this.extents = new b2Vec2
};
b2OBB.prototype.R = new b2Mat22;
b2OBB.prototype.center = new b2Vec2;
b2OBB.prototype.extents = new b2Vec2;var b2Pair = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Pair.prototype.__constructor = function() {
};
b2Pair.prototype.__varz = function() {
};
b2Pair.b2_nullProxy = b2Settings.USHRT_MAX;
b2Pair.e_pairBuffered = 1;
b2Pair.e_pairRemoved = 2;
b2Pair.e_pairFinal = 4;
b2Pair.prototype.userData = null;
b2Pair.prototype.proxy1 = null;
b2Pair.prototype.proxy2 = null;
b2Pair.prototype.next = null;
b2Pair.prototype.status = 0;
b2Pair.prototype.SetBuffered = function() {
  this.status |= b2Pair.e_pairBuffered
};
b2Pair.prototype.ClearBuffered = function() {
  this.status &= ~b2Pair.e_pairBuffered
};
b2Pair.prototype.IsBuffered = function() {
  return(this.status & b2Pair.e_pairBuffered) == b2Pair.e_pairBuffered
};
b2Pair.prototype.SetRemoved = function() {
  this.status |= b2Pair.e_pairRemoved
};
b2Pair.prototype.ClearRemoved = function() {
  this.status &= ~b2Pair.e_pairRemoved
};
b2Pair.prototype.IsRemoved = function() {
  return(this.status & b2Pair.e_pairRemoved) == b2Pair.e_pairRemoved
};
b2Pair.prototype.SetFinal = function() {
  this.status |= b2Pair.e_pairFinal
};
b2Pair.prototype.IsFinal = function() {
  return(this.status & b2Pair.e_pairFinal) == b2Pair.e_pairFinal
};var b2FixtureDef = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2FixtureDef.prototype.__constructor = function() {
  this.shape = null;
  this.userData = null;
  this.friction = 0.2;
  this.restitution = 0;
  this.density = 0;
  this.filter.categoryBits = 1;
  this.filter.maskBits = 65535;
  this.filter.groupIndex = 0;
  this.isSensor = false
};
b2FixtureDef.prototype.__varz = function() {
  this.filter = new b2FilterData
};
b2FixtureDef.prototype.shape = null;
b2FixtureDef.prototype.userData = null;
b2FixtureDef.prototype.friction = null;
b2FixtureDef.prototype.restitution = null;
b2FixtureDef.prototype.density = null;
b2FixtureDef.prototype.isSensor = null;
b2FixtureDef.prototype.filter = new b2FilterData;var b2PrismaticJoint = function() {
  b2Joint.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2PrismaticJoint.prototype, b2Joint.prototype);
b2PrismaticJoint.prototype._super = function() {
  b2Joint.prototype.__constructor.apply(this, arguments)
};
b2PrismaticJoint.prototype.__constructor = function(def) {
  this._super(def);
  var tMat;
  var tX;
  var tY;
  m_localAnchor1.SetV(def.localAnchorA);
  m_localAnchor2.SetV(def.localAnchorB);
  m_localXAxis1.SetV(def.localAxisA);
  this.m_localYAxis1.x = -m_localXAxis1.y;
  this.m_localYAxis1.y = m_localXAxis1.x;
  this.m_refAngle = def.referenceAngle;
  this.m_impulse.SetZero();
  this.m_motorMass = 0;
  this.m_motorImpulse = 0;
  this.m_lowerTranslation = def.lowerTranslation;
  this.m_upperTranslation = def.upperTranslation;
  this.m_maxMotorForce = def.maxMotorForce;
  this.m_motorSpeed = def.motorSpeed;
  this.m_enableLimit = def.enableLimit;
  this.m_enableMotor = def.enableMotor;
  this.m_limitState = e_inactiveLimit;
  this.m_axis.SetZero();
  this.m_perp.SetZero()
};
b2PrismaticJoint.prototype.__varz = function() {
  this.m_localYAxis1 = new b2Vec2;
  this.m_axis = new b2Vec2;
  this.m_perp = new b2Vec2;
  this.m_K = new b2Mat33;
  this.m_impulse = new b2Vec3
};
b2PrismaticJoint.prototype.m_localYAxis1 = new b2Vec2;
b2PrismaticJoint.prototype.m_refAngle = null;
b2PrismaticJoint.prototype.m_axis = new b2Vec2;
b2PrismaticJoint.prototype.m_perp = new b2Vec2;
b2PrismaticJoint.prototype.m_s1 = null;
b2PrismaticJoint.prototype.m_s2 = null;
b2PrismaticJoint.prototype.m_a1 = null;
b2PrismaticJoint.prototype.m_a2 = null;
b2PrismaticJoint.prototype.m_K = new b2Mat33;
b2PrismaticJoint.prototype.m_impulse = new b2Vec3;
b2PrismaticJoint.prototype.m_motorMass = null;
b2PrismaticJoint.prototype.m_motorImpulse = null;
b2PrismaticJoint.prototype.m_lowerTranslation = null;
b2PrismaticJoint.prototype.m_upperTranslation = null;
b2PrismaticJoint.prototype.m_maxMotorForce = null;
b2PrismaticJoint.prototype.m_motorSpeed = null;
b2PrismaticJoint.prototype.m_enableLimit = null;
b2PrismaticJoint.prototype.m_enableMotor = null;
b2PrismaticJoint.prototype.m_limitState = 0;
b2PrismaticJoint.prototype.GetAnchorA = function() {
  return m_bodyA.GetWorldPoint(m_localAnchor1)
};
b2PrismaticJoint.prototype.GetAnchorB = function() {
  return m_bodyB.GetWorldPoint(m_localAnchor2)
};
b2PrismaticJoint.prototype.GetReactionForce = function(inv_dt) {
  return new b2Vec2(inv_dt * (this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x), inv_dt * (this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y))
};
b2PrismaticJoint.prototype.GetReactionTorque = function(inv_dt) {
  return inv_dt * this.m_impulse.y
};
b2PrismaticJoint.prototype.GetJointTranslation = function() {
  var bA = m_bodyA;
  var bB = m_bodyB;
  var tMat;
  var p1 = bA.GetWorldPoint(m_localAnchor1);
  var p2 = bB.GetWorldPoint(m_localAnchor2);
  var dX = p2.x - p1.x;
  var dY = p2.y - p1.y;
  var axis = bA.GetWorldVector(m_localXAxis1);
  var translation = axis.x * dX + axis.y * dY;
  return translation
};
b2PrismaticJoint.prototype.GetJointSpeed = function() {
  var bA = m_bodyA;
  var bB = m_bodyB;
  var tMat;
  tMat = bA.m_xf.R;
  var r1X = m_localAnchor1.x - bA.m_sweep.localCenter.x;
  var r1Y = m_localAnchor1.y - bA.m_sweep.localCenter.y;
  var tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
  r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
  r1X = tX;
  tMat = bB.m_xf.R;
  var r2X = m_localAnchor2.x - bB.m_sweep.localCenter.x;
  var r2Y = m_localAnchor2.y - bB.m_sweep.localCenter.y;
  tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
  r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
  r2X = tX;
  var p1X = bA.m_sweep.c.x + r1X;
  var p1Y = bA.m_sweep.c.y + r1Y;
  var p2X = bB.m_sweep.c.x + r2X;
  var p2Y = bB.m_sweep.c.y + r2Y;
  var dX = p2X - p1X;
  var dY = p2Y - p1Y;
  var axis = bA.GetWorldVector(m_localXAxis1);
  var v1 = bA.m_linearVelocity;
  var v2 = bB.m_linearVelocity;
  var w1 = bA.m_angularVelocity;
  var w2 = bB.m_angularVelocity;
  var speed = dX * -w1 * axis.y + dY * w1 * axis.x + (axis.x * (v2.x + -w2 * r2Y - v1.x - -w1 * r1Y) + axis.y * (v2.y + w2 * r2X - v1.y - w1 * r1X));
  return speed
};
b2PrismaticJoint.prototype.IsLimitEnabled = function() {
  return this.m_enableLimit
};
b2PrismaticJoint.prototype.EnableLimit = function(flag) {
  m_bodyA.SetAwake(true);
  m_bodyB.SetAwake(true);
  this.m_enableLimit = flag
};
b2PrismaticJoint.prototype.GetLowerLimit = function() {
  return this.m_lowerTranslation
};
b2PrismaticJoint.prototype.GetUpperLimit = function() {
  return this.m_upperTranslation
};
b2PrismaticJoint.prototype.SetLimits = function(lower, upper) {
  m_bodyA.SetAwake(true);
  m_bodyB.SetAwake(true);
  this.m_lowerTranslation = lower;
  this.m_upperTranslation = upper
};
b2PrismaticJoint.prototype.IsMotorEnabled = function() {
  return this.m_enableMotor
};
b2PrismaticJoint.prototype.EnableMotor = function(flag) {
  m_bodyA.SetAwake(true);
  m_bodyB.SetAwake(true);
  this.m_enableMotor = flag
};
b2PrismaticJoint.prototype.SetMotorSpeed = function(speed) {
  m_bodyA.SetAwake(true);
  m_bodyB.SetAwake(true);
  this.m_motorSpeed = speed
};
b2PrismaticJoint.prototype.GetMotorSpeed = function() {
  return this.m_motorSpeed
};
b2PrismaticJoint.prototype.SetMaxMotorForce = function(force) {
  m_bodyA.SetAwake(true);
  m_bodyB.SetAwake(true);
  this.m_maxMotorForce = force
};
b2PrismaticJoint.prototype.GetMotorForce = function() {
  return this.m_motorImpulse
};var b2ContactID = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ContactID.prototype.__constructor = function() {
  this.features._m_id = this
};
b2ContactID.prototype.__varz = function() {
  this.features = new Features
};
b2ContactID.prototype.features = new Features;
b2ContactID.prototype._key = 0;
b2ContactID.prototype.Set = function(id) {
  key = id._key
};
b2ContactID.prototype.Copy = function() {
  var id = new b2ContactID;
  id.key = key;
  return id
};
b2ContactID.prototype.__defineSetter__("key", function() {
  return this._key
});
b2ContactID.prototype.__defineSetter__("key", function(value) {
  this._key = value;
  this.features._referenceEdge = this._key & 255;
  this.features._incidentEdge = (this._key & 65280) >> 8 & 255;
  this.features._incidentVertex = (this._key & 16711680) >> 16 & 255;
  this.features._flip = (this._key & 4278190080) >> 24 & 255
});var b2RevoluteJoint = function() {
  b2Joint.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2RevoluteJoint.prototype, b2Joint.prototype);
b2RevoluteJoint.prototype._super = function() {
  b2Joint.prototype.__constructor.apply(this, arguments)
};
b2RevoluteJoint.prototype.__constructor = function(def) {
  this._super(def);
  m_localAnchor1.SetV(def.localAnchorA);
  m_localAnchor2.SetV(def.localAnchorB);
  this.m_referenceAngle = def.referenceAngle;
  this.m_impulse.SetZero();
  this.m_motorImpulse = 0;
  this.m_lowerAngle = def.lowerAngle;
  this.m_upperAngle = def.upperAngle;
  this.m_maxMotorTorque = def.maxMotorTorque;
  this.m_motorSpeed = def.motorSpeed;
  this.m_enableLimit = def.enableLimit;
  this.m_enableMotor = def.enableMotor;
  this.m_limitState = e_inactiveLimit
};
b2RevoluteJoint.prototype.__varz = function() {
  this.K = new b2Mat22;
  this.K1 = new b2Mat22;
  this.K2 = new b2Mat22;
  this.K3 = new b2Mat22;
  this.impulse3 = new b2Vec3;
  this.impulse2 = new b2Vec2;
  this.reduced = new b2Vec2;
  this.m_impulse = new b2Vec3;
  this.m_mass = new b2Mat33
};
b2RevoluteJoint.tImpulse = new b2Vec2;
b2RevoluteJoint.prototype.K = new b2Mat22;
b2RevoluteJoint.prototype.K1 = new b2Mat22;
b2RevoluteJoint.prototype.K2 = new b2Mat22;
b2RevoluteJoint.prototype.K3 = new b2Mat22;
b2RevoluteJoint.prototype.impulse3 = new b2Vec3;
b2RevoluteJoint.prototype.impulse2 = new b2Vec2;
b2RevoluteJoint.prototype.reduced = new b2Vec2;
b2RevoluteJoint.prototype.m_impulse = new b2Vec3;
b2RevoluteJoint.prototype.m_motorImpulse = null;
b2RevoluteJoint.prototype.m_mass = new b2Mat33;
b2RevoluteJoint.prototype.m_motorMass = null;
b2RevoluteJoint.prototype.m_enableMotor = null;
b2RevoluteJoint.prototype.m_maxMotorTorque = null;
b2RevoluteJoint.prototype.m_motorSpeed = null;
b2RevoluteJoint.prototype.m_enableLimit = null;
b2RevoluteJoint.prototype.m_referenceAngle = null;
b2RevoluteJoint.prototype.m_lowerAngle = null;
b2RevoluteJoint.prototype.m_upperAngle = null;
b2RevoluteJoint.prototype.m_limitState = 0;
b2RevoluteJoint.prototype.GetAnchorA = function() {
  return m_bodyA.GetWorldPoint(m_localAnchor1)
};
b2RevoluteJoint.prototype.GetAnchorB = function() {
  return m_bodyB.GetWorldPoint(m_localAnchor2)
};
b2RevoluteJoint.prototype.GetReactionForce = function(inv_dt) {
  return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y)
};
b2RevoluteJoint.prototype.GetReactionTorque = function(inv_dt) {
  return inv_dt * this.m_impulse.z
};
b2RevoluteJoint.prototype.GetJointAngle = function() {
  return m_bodyB.m_sweep.a - m_bodyA.m_sweep.a - this.m_referenceAngle
};
b2RevoluteJoint.prototype.GetJointSpeed = function() {
  return m_bodyB.m_angularVelocity - m_bodyA.m_angularVelocity
};
b2RevoluteJoint.prototype.IsLimitEnabled = function() {
  return this.m_enableLimit
};
b2RevoluteJoint.prototype.EnableLimit = function(flag) {
  this.m_enableLimit = flag
};
b2RevoluteJoint.prototype.GetLowerLimit = function() {
  return this.m_lowerAngle
};
b2RevoluteJoint.prototype.GetUpperLimit = function() {
  return this.m_upperAngle
};
b2RevoluteJoint.prototype.SetLimits = function(lower, upper) {
  this.m_lowerAngle = lower;
  this.m_upperAngle = upper
};
b2RevoluteJoint.prototype.IsMotorEnabled = function() {
  m_bodyA.SetAwake(true);
  m_bodyB.SetAwake(true);
  return this.m_enableMotor
};
b2RevoluteJoint.prototype.EnableMotor = function(flag) {
  this.m_enableMotor = flag
};
b2RevoluteJoint.prototype.SetMotorSpeed = function(speed) {
  m_bodyA.SetAwake(true);
  m_bodyB.SetAwake(true);
  this.m_motorSpeed = speed
};
b2RevoluteJoint.prototype.GetMotorSpeed = function() {
  return this.m_motorSpeed
};
b2RevoluteJoint.prototype.SetMaxMotorTorque = function(torque) {
  this.m_maxMotorTorque = torque
};
b2RevoluteJoint.prototype.GetMotorTorque = function() {
  return this.m_maxMotorTorque
};var b2JointDef = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2JointDef.prototype.__constructor = function() {
  this.type = b2Joint.e_unknownJoint;
  this.userData = null;
  this.bodyA = null;
  this.bodyB = null;
  this.collideConnected = false
};
b2JointDef.prototype.__varz = function() {
};
b2JointDef.prototype.type = 0;
b2JointDef.prototype.userData = null;
b2JointDef.prototype.bodyA = null;
b2JointDef.prototype.bodyB = null;
b2JointDef.prototype.collideConnected = null;var b2Transform = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Transform.prototype.__constructor = function(pos, r) {
  if(pos) {
    this.position.SetV(pos);
    this.R.SetM(r)
  }
};
b2Transform.prototype.__varz = function() {
  this.position = new b2Vec2;
  this.R = new b2Mat22
};
b2Transform.prototype.position = new b2Vec2;
b2Transform.prototype.R = new b2Mat22;
b2Transform.prototype.Initialize = function(pos, r) {
  this.position.SetV(pos);
  this.R.SetM(r)
};
b2Transform.prototype.SetIdentity = function() {
  this.position.SetZero();
  this.R.SetIdentity()
};
b2Transform.prototype.Set = function(x) {
  this.position.SetV(x.position);
  this.R.SetM(x.R)
};
b2Transform.prototype.GetAngle = function() {
  return Math.atan2(this.R.col1.y, this.R.col1.x)
};var b2EdgeShape = function() {
  b2Shape.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2EdgeShape.prototype, b2Shape.prototype);
b2EdgeShape.prototype._super = function() {
  b2Shape.prototype.__constructor.apply(this, arguments)
};
b2EdgeShape.prototype.__constructor = function(v1, v2) {
  this._super();
  m_type = e_edgeShape;
  m_prevEdge = null;
  m_nextEdge = null;
  m_v1 = v1;
  m_v2 = v2;
  m_direction.Set(m_v2.x - m_v1.x, m_v2.y - m_v1.y);
  m_length = m_direction.Normalize();
  m_normal.Set(m_direction.y, -m_direction.x);
  m_coreV1.Set(-b2Settings.b2_toiSlop * (m_normal.x - m_direction.x) + m_v1.x, -b2Settings.b2_toiSlop * (m_normal.y - m_direction.y) + m_v1.y);
  m_coreV2.Set(-b2Settings.b2_toiSlop * (m_normal.x + m_direction.x) + m_v2.x, -b2Settings.b2_toiSlop * (m_normal.y + m_direction.y) + m_v2.y);
  m_cornerDir1 = m_normal;
  m_cornerDir2.Set(-m_normal.x, -m_normal.y)
};
b2EdgeShape.prototype.__varz = function() {
  this.s_supportVec = new b2Vec2
};
b2EdgeShape.prototype.s_supportVec = new b2Vec2;
b2EdgeShape.prototype.TestPoint = function(transform, p) {
  return false
};
b2EdgeShape.prototype.RayCast = function(output, input, transform) {
  var tMat;
  var rX = input.p2.x - input.p1.x;
  var rY = input.p2.y - input.p1.y;
  tMat = transform.R;
  var v1X = transform.position.x + (tMat.col1.x * m_v1.x + tMat.col2.x * m_v1.y);
  var v1Y = transform.position.y + (tMat.col1.y * m_v1.x + tMat.col2.y * m_v1.y);
  var nX = transform.position.y + (tMat.col1.y * m_v2.x + tMat.col2.y * m_v2.y) - v1Y;
  var nY = -(transform.position.x + (tMat.col1.x * m_v2.x + tMat.col2.x * m_v2.y) - v1X);
  var k_slop = 100 * Number.MIN_VALUE;
  var denom = -(rX * nX + rY * nY);
  if(denom > k_slop) {
    var bX = input.p1.x - v1X;
    var bY = input.p1.y - v1Y;
    var a = bX * nX + bY * nY;
    if(0 <= a && a <= input.maxFraction * denom) {
      var mu2 = -rX * bY + rY * bX;
      if(-k_slop * denom <= mu2 && mu2 <= denom * (1 + k_slop)) {
        a /= denom;
        output.fraction = a;
        var nLen = Math.sqrt(nX * nX + nY * nY);
        output.normal.x = nX / nLen;
        output.normal.y = nY / nLen;
        return true
      }
    }
  }
  return false
};
b2EdgeShape.prototype.ComputeAABB = function(aabb, transform) {
  var tMat = transform.R;
  var v1X = transform.position.x + (tMat.col1.x * m_v1.x + tMat.col2.x * m_v1.y);
  var v1Y = transform.position.y + (tMat.col1.y * m_v1.x + tMat.col2.y * m_v1.y);
  var v2X = transform.position.x + (tMat.col1.x * m_v2.x + tMat.col2.x * m_v2.y);
  var v2Y = transform.position.y + (tMat.col1.y * m_v2.x + tMat.col2.y * m_v2.y);
  if(v1X < v2X) {
    aabb.lowerBound.x = v1X;
    aabb.upperBound.x = v2X
  }else {
    aabb.lowerBound.x = v2X;
    aabb.upperBound.x = v1X
  }
  if(v1Y < v2Y) {
    aabb.lowerBound.y = v1Y;
    aabb.upperBound.y = v2Y
  }else {
    aabb.lowerBound.y = v2Y;
    aabb.upperBound.y = v1Y
  }
};
b2EdgeShape.prototype.ComputeMass = function(massData, density) {
  massData.mass = 0;
  massData.center.SetV(m_v1);
  massData.I = 0
};
b2EdgeShape.prototype.ComputeSubmergedArea = function(normal, offset, xf, c) {
  var v0 = new b2Vec2(normal.x * offset, normal.y * offset);
  var v1 = b2Math.MulX(xf, m_v1);
  var v2 = b2Math.MulX(xf, m_v2);
  var d1 = b2Math.Dot(normal, v1) - offset;
  var d2 = b2Math.Dot(normal, v2) - offset;
  if(d1 > 0) {
    if(d2 > 0) {
      return 0
    }else {
      v1.x = -d2 / (d1 - d2) * v1.x + d1 / (d1 - d2) * v2.x;
      v1.y = -d2 / (d1 - d2) * v1.y + d1 / (d1 - d2) * v2.y
    }
  }else {
    if(d2 > 0) {
      v2.x = -d2 / (d1 - d2) * v1.x + d1 / (d1 - d2) * v2.x;
      v2.y = -d2 / (d1 - d2) * v1.y + d1 / (d1 - d2) * v2.y
    }else {
    }
  }
  c.x = (v0.x + v1.x + v2.x) / 3;
  c.y = (v0.y + v1.y + v2.y) / 3;
  return 0.5 * ((v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x))
};
b2EdgeShape.prototype.GetLength = function() {
  return m_length
};
b2EdgeShape.prototype.GetVertex1 = function() {
  return m_v1
};
b2EdgeShape.prototype.GetVertex2 = function() {
  return m_v2
};
b2EdgeShape.prototype.GetCoreVertex1 = function() {
  return m_coreV1
};
b2EdgeShape.prototype.GetCoreVertex2 = function() {
  return m_coreV2
};
b2EdgeShape.prototype.GetNormalVector = function() {
  return m_normal
};
b2EdgeShape.prototype.GetDirectionVector = function() {
  return m_direction
};
b2EdgeShape.prototype.GetCorner1Vector = function() {
  return m_cornerDir1
};
b2EdgeShape.prototype.GetCorner2Vector = function() {
  return m_cornerDir2
};
b2EdgeShape.prototype.Corner1IsConvex = function() {
  return m_cornerConvex1
};
b2EdgeShape.prototype.Corner2IsConvex = function() {
  return m_cornerConvex2
};
b2EdgeShape.prototype.GetFirstVertex = function(xf) {
  var tMat = xf.R;
  return new b2Vec2(xf.position.x + (tMat.col1.x * m_coreV1.x + tMat.col2.x * m_coreV1.y), xf.position.y + (tMat.col1.y * m_coreV1.x + tMat.col2.y * m_coreV1.y))
};
b2EdgeShape.prototype.GetNextEdge = function() {
  return m_nextEdge
};
b2EdgeShape.prototype.GetPrevEdge = function() {
  return m_prevEdge
};
b2EdgeShape.prototype.Support = function(xf, dX, dY) {
  var tMat = xf.R;
  var v1X = xf.position.x + (tMat.col1.x * m_coreV1.x + tMat.col2.x * m_coreV1.y);
  var v1Y = xf.position.y + (tMat.col1.y * m_coreV1.x + tMat.col2.y * m_coreV1.y);
  var v2X = xf.position.x + (tMat.col1.x * m_coreV2.x + tMat.col2.x * m_coreV2.y);
  var v2Y = xf.position.y + (tMat.col1.y * m_coreV2.x + tMat.col2.y * m_coreV2.y);
  if(v1X * dX + v1Y * dY > v2X * dX + v2Y * dY) {
    this.s_supportVec.x = v1X;
    this.s_supportVec.y = v1Y
  }else {
    this.s_supportVec.x = v2X;
    this.s_supportVec.y = v2Y
  }
  return this.s_supportVec
};var b2BuoyancyController = function() {
  b2Controller.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2BuoyancyController.prototype, b2Controller.prototype);
b2BuoyancyController.prototype._super = function() {
  b2Controller.prototype.__constructor.apply(this, arguments)
};
b2BuoyancyController.prototype.__constructor = function() {
};
b2BuoyancyController.prototype.__varz = function() {
  this.normal = new b2Vec2(0, -1);
  this.velocity = new b2Vec2(0, 0)
};
b2BuoyancyController.prototype.normal = new b2Vec2(0, -1);
b2BuoyancyController.prototype.offset = 0;
b2BuoyancyController.prototype.density = 0;
b2BuoyancyController.prototype.velocity = new b2Vec2(0, 0);
b2BuoyancyController.prototype.linearDrag = 2;
b2BuoyancyController.prototype.angularDrag = 1;
b2BuoyancyController.prototype.useDensity = false;
b2BuoyancyController.prototype.useWorldGravity = true;
b2BuoyancyController.prototype.gravity = null;
b2BuoyancyController.prototype.Step = function(step) {
  if(!m_bodyList) {
    return
  }
  if(this.useWorldGravity) {
    this.gravity = this.GetWorld().GetGravity().Copy()
  }
  for(var i = m_bodyList;i;i = i.nextBody) {
    var body = i.body;
    if(body.IsAwake() == false) {
      continue
    }
    var areac = new b2Vec2;
    var massc = new b2Vec2;
    var area = 0;
    var mass = 0;
    for(var fixture = body.GetFixtureList();fixture;fixture = fixture.GetNext()) {
      var sc = new b2Vec2;
      var sarea = fixture.GetShape().ComputeSubmergedArea(this.normal, this.offset, body.GetTransform(), sc);
      area += sarea;
      areac.x += sarea * sc.x;
      areac.y += sarea * sc.y;
      var shapeDensity;
      if(this.useDensity) {
        shapeDensity = 1
      }else {
        shapeDensity = 1
      }
      mass += sarea * shapeDensity;
      massc.x += sarea * sc.x * shapeDensity;
      massc.y += sarea * sc.y * shapeDensity
    }
    areac.x /= area;
    areac.y /= area;
    massc.x /= mass;
    massc.y /= mass;
    if(area < Number.MIN_VALUE) {
      continue
    }
    var buoyancyForce = this.gravity.GetNegative();
    buoyancyForce.Multiply(this.density * area);
    body.ApplyForce(buoyancyForce, massc);
    var dragForce = body.GetLinearVelocityFromWorldPoint(areac);
    dragForce.Subtract(this.velocity);
    dragForce.Multiply(-this.linearDrag * area);
    body.ApplyForce(dragForce, areac);
    body.ApplyTorque(-body.GetInertia() / body.GetMass() * area * body.GetAngularVelocity() * this.angularDrag)
  }
};
b2BuoyancyController.prototype.Draw = function(debugDraw) {
  var r = 1E3;
  var p1 = new b2Vec2;
  var p2 = new b2Vec2;
  p1.x = this.normal.x * this.offset + this.normal.y * r;
  p1.y = this.normal.y * this.offset - this.normal.x * r;
  p2.x = this.normal.x * this.offset - this.normal.y * r;
  p2.y = this.normal.y * this.offset + this.normal.x * r;
  var color = new b2Color(0, 0, 1);
  debugDraw.DrawSegment(p1, p2, color)
};var b2LineJointDef = function() {
  b2JointDef.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2LineJointDef.prototype, b2JointDef.prototype);
b2LineJointDef.prototype._super = function() {
  b2JointDef.prototype.__constructor.apply(this, arguments)
};
b2LineJointDef.prototype.__constructor = function() {
  this.type = b2Joint.e_lineJoint;
  this.localAxisA.Set(1, 0);
  this.enableLimit = false;
  this.lowerTranslation = 0;
  this.upperTranslation = 0;
  this.enableMotor = false;
  this.maxMotorForce = 0;
  this.motorSpeed = 0
};
b2LineJointDef.prototype.__varz = function() {
  this.localAnchorA = new b2Vec2;
  this.localAnchorB = new b2Vec2;
  this.localAxisA = new b2Vec2
};
b2LineJointDef.prototype.localAnchorA = new b2Vec2;
b2LineJointDef.prototype.localAnchorB = new b2Vec2;
b2LineJointDef.prototype.localAxisA = new b2Vec2;
b2LineJointDef.prototype.enableLimit = null;
b2LineJointDef.prototype.lowerTranslation = null;
b2LineJointDef.prototype.upperTranslation = null;
b2LineJointDef.prototype.enableMotor = null;
b2LineJointDef.prototype.maxMotorForce = null;
b2LineJointDef.prototype.motorSpeed = null;
b2LineJointDef.prototype.Initialize = function(bA, bB, anchor, axis) {
  this.bodyA = bA;
  this.bodyB = bB;
  this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
  this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
  this.localAxisA = this.bodyA.GetLocalVector(axis)
};var b2DistanceJoint = function() {
  b2Joint.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2DistanceJoint.prototype, b2Joint.prototype);
b2DistanceJoint.prototype._super = function() {
  b2Joint.prototype.__constructor.apply(this, arguments)
};
b2DistanceJoint.prototype.__constructor = function(def) {
  this._super(def);
  var tMat;
  var tX;
  var tY;
  this.m_localAnchor1.SetV(def.localAnchorA);
  this.m_localAnchor2.SetV(def.localAnchorB);
  this.m_length = def.length;
  this.m_frequencyHz = def.frequencyHz;
  this.m_dampingRatio = def.dampingRatio;
  this.m_impulse = 0;
  this.m_gamma = 0;
  this.m_bias = 0
};
b2DistanceJoint.prototype.__varz = function() {
  this.m_localAnchor1 = new b2Vec2;
  this.m_localAnchor2 = new b2Vec2;
  this.m_u = new b2Vec2
};
b2DistanceJoint.prototype.m_localAnchor1 = new b2Vec2;
b2DistanceJoint.prototype.m_localAnchor2 = new b2Vec2;
b2DistanceJoint.prototype.m_u = new b2Vec2;
b2DistanceJoint.prototype.m_frequencyHz = null;
b2DistanceJoint.prototype.m_dampingRatio = null;
b2DistanceJoint.prototype.m_gamma = null;
b2DistanceJoint.prototype.m_bias = null;
b2DistanceJoint.prototype.m_impulse = null;
b2DistanceJoint.prototype.m_mass = null;
b2DistanceJoint.prototype.m_length = null;
b2DistanceJoint.prototype.GetAnchorA = function() {
  return m_bodyA.GetWorldPoint(this.m_localAnchor1)
};
b2DistanceJoint.prototype.GetAnchorB = function() {
  return m_bodyB.GetWorldPoint(this.m_localAnchor2)
};
b2DistanceJoint.prototype.GetReactionForce = function(inv_dt) {
  return new b2Vec2(inv_dt * this.m_impulse * this.m_u.x, inv_dt * this.m_impulse * this.m_u.y)
};
b2DistanceJoint.prototype.GetReactionTorque = function(inv_dt) {
  return 0
};
b2DistanceJoint.prototype.GetLength = function() {
  return this.m_length
};
b2DistanceJoint.prototype.SetLength = function(length) {
  this.m_length = length
};
b2DistanceJoint.prototype.GetFrequency = function() {
  return this.m_frequencyHz
};
b2DistanceJoint.prototype.SetFrequency = function(hz) {
  this.m_frequencyHz = hz
};
b2DistanceJoint.prototype.GetDampingRatio = function() {
  return this.m_dampingRatio
};
b2DistanceJoint.prototype.SetDampingRatio = function(ratio) {
  this.m_dampingRatio = ratio
};var b2Body = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Body.prototype.__constructor = function(bd, world) {
  m_flags = 0;
  if(bd.bullet) {
    m_flags |= e_bulletFlag
  }
  if(bd.fixedRotation) {
    m_flags |= e_fixedRotationFlag
  }
  if(bd.allowSleep) {
    m_flags |= e_allowSleepFlag
  }
  if(bd.awake) {
    m_flags |= e_awakeFlag
  }
  if(bd.active) {
    m_flags |= e_activeFlag
  }
  m_world = world;
  m_xf.position.SetV(bd.position);
  m_xf.R.Set(bd.angle);
  m_sweep.localCenter.SetZero();
  m_sweep.t0 = 1;
  m_sweep.a0 = m_sweep.a = bd.angle;
  var tMat = m_xf.R;
  var tVec = m_sweep.localCenter;
  m_sweep.c.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
  m_sweep.c.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
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
  m_force.Set(0, 0);
  m_torque = 0;
  m_sleepTime = 0;
  m_type = bd.type;
  if(m_type == b2Body.b2_dynamicBody) {
    m_mass = 1;
    m_invMass = 1
  }else {
    m_mass = 0;
    m_invMass = 0
  }
  m_I = 0;
  m_invI = 0;
  m_inertiaScale = bd.inertiaScale;
  this.m_userData = bd.userData;
  m_fixtureList = null;
  m_fixtureCount = 0
};
b2Body.prototype.__varz = function() {
};
b2Body.b2_staticBody = 0;
b2Body.b2_kinematicBody = 1;
b2Body.b2_dynamicBody = 2;
b2Body.s_xf1 = new b2Transform;
b2Body.prototype.m_userData = null;
b2Body.prototype.connectEdges = function(s1, s2, angle1) {
  var angle2 = Math.atan2(s2.GetDirectionVector().y, s2.GetDirectionVector().x);
  var coreOffset = Math.tan((angle2 - angle1) * 0.5);
  var core = b2Math.MulFV(coreOffset, s2.GetDirectionVector());
  core = b2Math.SubtractVV(core, s2.GetNormalVector());
  core = b2Math.MulFV(b2Settings.b2_toiSlop, core);
  core = b2Math.AddVV(core, s2.GetVertex1());
  var cornerDir = b2Math.AddVV(s1.GetDirectionVector(), s2.GetDirectionVector());
  cornerDir.Normalize();
  var convex = b2Math.Dot(s1.GetDirectionVector(), s2.GetNormalVector()) > 0;
  s1.SetNextEdge(s2, core, cornerDir, convex);
  s2.SetPrevEdge(s1, core, cornerDir, convex);
  return angle2
};
b2Body.prototype.CreateFixture = function(def) {
  if(m_world.IsLocked() == true) {
    return null
  }
  var fixture = new b2Fixture;
  fixture.Create(this, m_xf, def);
  if(m_flags & e_activeFlag) {
    var broadPhase = m_world.m_contactManager.m_broadPhase;
    fixture.CreateProxy(broadPhase, m_xf)
  }
  fixture.m_next = m_fixtureList;
  m_fixtureList = fixture;
  ++m_fixtureCount;
  fixture.m_body = this;
  if(fixture.m_density > 0) {
    this.ResetMassData()
  }
  m_world.m_flags |= b2World.e_newFixture;
  return fixture
};
b2Body.prototype.CreateFixture2 = function(shape, density) {
  var def = new b2FixtureDef;
  def.shape = shape;
  def.density = density;
  return this.CreateFixture(def)
};
b2Body.prototype.DestroyFixture = function(fixture) {
  if(m_world.IsLocked() == true) {
    return
  }
  var node = m_fixtureList;
  var ppF = null;
  var found = false;
  while(node != null) {
    if(node == fixture) {
      if(ppF) {
        ppF.m_next = fixture.m_next
      }else {
        m_fixtureList = fixture.m_next
      }
      found = true;
      break
    }
    ppF = node;
    node = node.m_next
  }
  var edge = m_contactList;
  while(edge) {
    var c = edge.contact;
    edge = edge.next;
    var fixtureA = c.GetFixtureA();
    var fixtureB = c.GetFixtureB();
    if(fixture == fixtureA || fixture == fixtureB) {
      m_world.m_contactManager.Destroy(c)
    }
  }
  if(m_flags & e_activeFlag) {
    var broadPhase = m_world.m_contactManager.m_broadPhase;
    fixture.DestroyProxy(broadPhase)
  }else {
  }
  fixture.Destroy();
  fixture.m_body = null;
  fixture.m_next = null;
  --m_fixtureCount;
  this.ResetMassData()
};
b2Body.prototype.SetPositionAndAngle = function(position, angle) {
  var f;
  if(m_world.IsLocked() == true) {
    return
  }
  m_xf.R.Set(angle);
  m_xf.position.SetV(position);
  var tMat = m_xf.R;
  var tVec = m_sweep.localCenter;
  m_sweep.c.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
  m_sweep.c.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
  m_sweep.c.x += m_xf.position.x;
  m_sweep.c.y += m_xf.position.y;
  m_sweep.c0.SetV(m_sweep.c);
  m_sweep.a0 = m_sweep.a = angle;
  var broadPhase = m_world.m_contactManager.m_broadPhase;
  for(f = m_fixtureList;f;f = f.m_next) {
    f.Synchronize(broadPhase, m_xf, m_xf)
  }
  m_world.m_contactManager.FindNewContacts()
};
b2Body.prototype.SetTransform = function(xf) {
  this.SetPositionAndAngle(xf.position, xf.GetAngle())
};
b2Body.prototype.GetTransform = function() {
  return m_xf
};
b2Body.prototype.GetPosition = function() {
  return m_xf.position
};
b2Body.prototype.SetPosition = function(position) {
  this.SetPositionAndAngle(position, this.GetAngle())
};
b2Body.prototype.GetAngle = function() {
  return m_sweep.a
};
b2Body.prototype.SetAngle = function(angle) {
  this.SetPositionAndAngle(this.GetPosition(), angle)
};
b2Body.prototype.GetWorldCenter = function() {
  return m_sweep.c
};
b2Body.prototype.GetLocalCenter = function() {
  return m_sweep.localCenter
};
b2Body.prototype.SetLinearVelocity = function(v) {
  if(m_type == b2Body.b2_staticBody) {
    return
  }
  m_linearVelocity.SetV(v)
};
b2Body.prototype.GetLinearVelocity = function() {
  return m_linearVelocity
};
b2Body.prototype.SetAngularVelocity = function(omega) {
  if(m_type == b2Body.b2_staticBody) {
    return
  }
  m_angularVelocity = omega
};
b2Body.prototype.GetAngularVelocity = function() {
  return m_angularVelocity
};
b2Body.prototype.GetDefinition = function() {
  var bd = new b2BodyDef;
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
  return bd
};
b2Body.prototype.ApplyForce = function(force, point) {
  if(m_type != b2Body.b2_dynamicBody) {
    return
  }
  if(this.IsAwake() == false) {
    this.SetAwake(true)
  }
  m_force.x += force.x;
  m_force.y += force.y;
  m_torque += (point.x - m_sweep.c.x) * force.y - (point.y - m_sweep.c.y) * force.x
};
b2Body.prototype.ApplyTorque = function(torque) {
  if(m_type != b2Body.b2_dynamicBody) {
    return
  }
  if(this.IsAwake() == false) {
    this.SetAwake(true)
  }
  m_torque += torque
};
b2Body.prototype.ApplyImpulse = function(impulse, point) {
  if(m_type != b2Body.b2_dynamicBody) {
    return
  }
  if(this.IsAwake() == false) {
    this.SetAwake(true)
  }
  m_linearVelocity.x += m_invMass * impulse.x;
  m_linearVelocity.y += m_invMass * impulse.y;
  m_angularVelocity += m_invI * ((point.x - m_sweep.c.x) * impulse.y - (point.y - m_sweep.c.y) * impulse.x)
};
b2Body.prototype.Split = function(callback) {
  var linearVelocity = this.GetLinearVelocity().Copy();
  var angularVelocity = this.GetAngularVelocity();
  var center = this.GetWorldCenter();
  var body1 = this;
  var body2 = m_world.CreateBody(this.GetDefinition());
  var prev;
  for(var f = body1.m_fixtureList;f;) {
    if(callback(f)) {
      var next = f.m_next;
      if(prev) {
        prev.m_next = next
      }else {
        body1.m_fixtureList = next
      }
      body1.m_fixtureCount--;
      f.m_next = body2.m_fixtureList;
      body2.m_fixtureList = f;
      body2.m_fixtureCount++;
      f.m_body = body2;
      f = next
    }else {
      prev = f;
      f = f.m_next
    }
  }
  body1.ResetMassData();
  body2.ResetMassData();
  var center1 = body1.GetWorldCenter();
  var center2 = body2.GetWorldCenter();
  var velocity1 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center1, center)));
  var velocity2 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center2, center)));
  body1.SetLinearVelocity(velocity1);
  body2.SetLinearVelocity(velocity2);
  body1.SetAngularVelocity(angularVelocity);
  body2.SetAngularVelocity(angularVelocity);
  body1.SynchronizeFixtures();
  body2.SynchronizeFixtures();
  return body2
};
b2Body.prototype.Merge = function(other) {
  var f;
  for(f = other.m_fixtureList;f;) {
    var next = f.m_next;
    other.m_fixtureCount--;
    f.m_next = m_fixtureList;
    m_fixtureList = f;
    m_fixtureCount++;
    f.m_body = body2;
    f = next
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
  SynchronizeFixtures()
};
b2Body.prototype.GetMass = function() {
  return m_mass
};
b2Body.prototype.GetInertia = function() {
  return m_I
};
b2Body.prototype.GetMassData = function(data) {
  data.mass = m_mass;
  data.I = m_I;
  data.center.SetV(m_sweep.localCenter)
};
b2Body.prototype.SetMassData = function(massData) {
  b2Settings.b2Assert(m_world.IsLocked() == false);
  if(m_world.IsLocked() == true) {
    return
  }
  if(m_type != b2Body.b2_dynamicBody) {
    return
  }
  m_invMass = 0;
  m_I = 0;
  m_invI = 0;
  m_mass = massData.mass;
  if(m_mass <= 0) {
    m_mass = 1
  }
  m_invMass = 1 / m_mass;
  if(massData.I > 0 && (m_flags & e_fixedRotationFlag) == 0) {
    m_I = massData.I - m_mass * (massData.center.x * massData.center.x + massData.center.y * massData.center.y);
    m_invI = 1 / m_I
  }
  var oldCenter = m_sweep.c.Copy();
  m_sweep.localCenter.SetV(massData.center);
  m_sweep.c0.SetV(b2Math.MulX(m_xf, m_sweep.localCenter));
  m_sweep.c.SetV(m_sweep.c0);
  m_linearVelocity.x += m_angularVelocity * -(m_sweep.c.y - oldCenter.y);
  m_linearVelocity.y += m_angularVelocity * +(m_sweep.c.x - oldCenter.x)
};
b2Body.prototype.ResetMassData = function() {
  m_mass = 0;
  m_invMass = 0;
  m_I = 0;
  m_invI = 0;
  m_sweep.localCenter.SetZero();
  if(m_type == b2Body.b2_staticBody || m_type == b2Body.b2_kinematicBody) {
    return
  }
  var center = b2Vec2.Make(0, 0);
  for(var f = m_fixtureList;f;f = f.m_next) {
    if(f.m_density == 0) {
      continue
    }
    var massData = f.GetMassData();
    m_mass += massData.mass;
    center.x += massData.center.x * massData.mass;
    center.y += massData.center.y * massData.mass;
    m_I += massData.I
  }
  if(m_mass > 0) {
    m_invMass = 1 / m_mass;
    center.x *= m_invMass;
    center.y *= m_invMass
  }else {
    m_mass = 1;
    m_invMass = 1
  }
  if(m_I > 0 && (m_flags & e_fixedRotationFlag) == 0) {
    m_I -= m_mass * (center.x * center.x + center.y * center.y);
    m_I *= m_inertiaScale;
    b2Settings.b2Assert(m_I > 0);
    m_invI = 1 / m_I
  }else {
    m_I = 0;
    m_invI = 0
  }
  var oldCenter = m_sweep.c.Copy();
  m_sweep.localCenter.SetV(center);
  m_sweep.c0.SetV(b2Math.MulX(m_xf, m_sweep.localCenter));
  m_sweep.c.SetV(m_sweep.c0);
  m_linearVelocity.x += m_angularVelocity * -(m_sweep.c.y - oldCenter.y);
  m_linearVelocity.y += m_angularVelocity * +(m_sweep.c.x - oldCenter.x)
};
b2Body.prototype.GetWorldPoint = function(localPoint) {
  var A = m_xf.R;
  var u = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
  u.x += m_xf.position.x;
  u.y += m_xf.position.y;
  return u
};
b2Body.prototype.GetWorldVector = function(localVector) {
  return b2Math.MulMV(m_xf.R, localVector)
};
b2Body.prototype.GetLocalPoint = function(worldPoint) {
  return b2Math.MulXT(m_xf, worldPoint)
};
b2Body.prototype.GetLocalVector = function(worldVector) {
  return b2Math.MulTMV(m_xf.R, worldVector)
};
b2Body.prototype.GetLinearVelocityFromWorldPoint = function(worldPoint) {
  return new b2Vec2(m_linearVelocity.x - m_angularVelocity * (worldPoint.y - m_sweep.c.y), m_linearVelocity.y + m_angularVelocity * (worldPoint.x - m_sweep.c.x))
};
b2Body.prototype.GetLinearVelocityFromLocalPoint = function(localPoint) {
  var A = m_xf.R;
  var worldPoint = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
  worldPoint.x += m_xf.position.x;
  worldPoint.y += m_xf.position.y;
  return new b2Vec2(m_linearVelocity.x - m_angularVelocity * (worldPoint.y - m_sweep.c.y), m_linearVelocity.y + m_angularVelocity * (worldPoint.x - m_sweep.c.x))
};
b2Body.prototype.GetLinearDamping = function() {
  return m_linearDamping
};
b2Body.prototype.SetLinearDamping = function(linearDamping) {
  m_linearDamping = linearDamping
};
b2Body.prototype.GetAngularDamping = function() {
  return m_angularDamping
};
b2Body.prototype.SetAngularDamping = function(angularDamping) {
  m_angularDamping = angularDamping
};
b2Body.prototype.SetType = function(type) {
  if(m_type == type) {
    return
  }
  m_type = type;
  this.ResetMassData();
  if(m_type == b2Body.b2_staticBody) {
    m_linearVelocity.SetZero();
    m_angularVelocity = 0
  }
  this.SetAwake(true);
  m_force.SetZero();
  m_torque = 0;
  for(var ce = m_contactList;ce;ce = ce.next) {
    ce.contact.FlagForFiltering()
  }
};
b2Body.prototype.GetType = function() {
  return m_type
};
b2Body.prototype.SetBullet = function(flag) {
  if(flag) {
    m_flags |= e_bulletFlag
  }else {
    m_flags &= ~e_bulletFlag
  }
};
b2Body.prototype.IsBullet = function() {
  return(m_flags & e_bulletFlag) == e_bulletFlag
};
b2Body.prototype.SetSleepingAllowed = function(flag) {
  if(flag) {
    m_flags |= e_allowSleepFlag
  }else {
    m_flags &= ~e_allowSleepFlag;
    this.SetAwake(true)
  }
};
b2Body.prototype.SetAwake = function(flag) {
  if(flag) {
    m_flags |= e_awakeFlag;
    m_sleepTime = 0
  }else {
    m_flags &= ~e_awakeFlag;
    m_sleepTime = 0;
    m_linearVelocity.SetZero();
    m_angularVelocity = 0;
    m_force.SetZero();
    m_torque = 0
  }
};
b2Body.prototype.IsAwake = function() {
  return(m_flags & e_awakeFlag) == e_awakeFlag
};
b2Body.prototype.SetFixedRotation = function(fixed) {
  if(fixed) {
    m_flags |= e_fixedRotationFlag
  }else {
    m_flags &= ~e_fixedRotationFlag
  }
  this.ResetMassData()
};
b2Body.prototype.IsFixedRotation = function() {
  return(m_flags & e_fixedRotationFlag) == e_fixedRotationFlag
};
b2Body.prototype.SetActive = function(flag) {
  if(flag == this.IsActive()) {
    return
  }
  var broadPhase;
  var f;
  if(flag) {
    m_flags |= e_activeFlag;
    broadPhase = m_world.m_contactManager.m_broadPhase;
    for(f = m_fixtureList;f;f = f.m_next) {
      f.CreateProxy(broadPhase, m_xf)
    }
  }else {
    m_flags &= ~e_activeFlag;
    broadPhase = m_world.m_contactManager.m_broadPhase;
    for(f = m_fixtureList;f;f = f.m_next) {
      f.DestroyProxy(broadPhase)
    }
    var ce = m_contactList;
    while(ce) {
      var ce0 = ce;
      ce = ce.next;
      m_world.m_contactManager.Destroy(ce0.contact)
    }
    m_contactList = null
  }
};
b2Body.prototype.IsActive = function() {
  return(m_flags & e_activeFlag) == e_activeFlag
};
b2Body.prototype.IsSleepingAllowed = function() {
  return(m_flags & e_allowSleepFlag) == e_allowSleepFlag
};
b2Body.prototype.GetFixtureList = function() {
  return m_fixtureList
};
b2Body.prototype.GetJointList = function() {
  return m_jointList
};
b2Body.prototype.GetControllerList = function() {
  return m_controllerList
};
b2Body.prototype.GetContactList = function() {
  return m_contactList
};
b2Body.prototype.GetNext = function() {
  return m_next
};
b2Body.prototype.GetUserData = function() {
  return this.m_userData
};
b2Body.prototype.SetUserData = function(data) {
  this.m_userData = data
};
b2Body.prototype.GetWorld = function() {
  return m_world
};var b2PulleyJointDef = function() {
  b2JointDef.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2PulleyJointDef.prototype, b2JointDef.prototype);
b2PulleyJointDef.prototype._super = function() {
  b2JointDef.prototype.__constructor.apply(this, arguments)
};
b2PulleyJointDef.prototype.__constructor = function() {
  this.type = b2Joint.e_pulleyJoint;
  this.groundAnchorA.Set(-1, 1);
  this.groundAnchorB.Set(1, 1);
  this.localAnchorA.Set(-1, 0);
  this.localAnchorB.Set(1, 0);
  this.lengthA = 0;
  this.maxLengthA = 0;
  this.lengthB = 0;
  this.maxLengthB = 0;
  this.ratio = 1;
  this.collideConnected = true
};
b2PulleyJointDef.prototype.__varz = function() {
  this.groundAnchorA = new b2Vec2;
  this.groundAnchorB = new b2Vec2;
  this.localAnchorA = new b2Vec2;
  this.localAnchorB = new b2Vec2
};
b2PulleyJointDef.prototype.groundAnchorA = new b2Vec2;
b2PulleyJointDef.prototype.groundAnchorB = new b2Vec2;
b2PulleyJointDef.prototype.localAnchorA = new b2Vec2;
b2PulleyJointDef.prototype.localAnchorB = new b2Vec2;
b2PulleyJointDef.prototype.lengthA = null;
b2PulleyJointDef.prototype.maxLengthA = null;
b2PulleyJointDef.prototype.lengthB = null;
b2PulleyJointDef.prototype.maxLengthB = null;
b2PulleyJointDef.prototype.ratio = null;
b2PulleyJointDef.prototype.Initialize = function(bA, bB, gaA, gaB, anchorA, anchorB, r) {
  this.bodyA = bA;
  this.bodyB = bB;
  this.groundAnchorA.SetV(gaA);
  this.groundAnchorB.SetV(gaB);
  this.localAnchorA = this.bodyA.GetLocalPoint(anchorA);
  this.localAnchorB = this.bodyB.GetLocalPoint(anchorB);
  var d1X = anchorA.x - gaA.x;
  var d1Y = anchorA.y - gaA.y;
  this.lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y);
  var d2X = anchorB.x - gaB.x;
  var d2Y = anchorB.y - gaB.y;
  this.lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y);
  this.ratio = r;
  var C = this.lengthA + this.ratio * this.lengthB;
  this.maxLengthA = C - this.ratio * b2PulleyJoint.b2_minPulleyLength;
  this.maxLengthB = (C - b2PulleyJoint.b2_minPulleyLength) / this.ratio
};var b2ContactImpulse = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ContactImpulse.prototype.__constructor = function() {
};
b2ContactImpulse.prototype.__varz = function() {
  this.normalImpulses = new Array(b2Settings.b2_maxManifoldPoints);
  this.tangentImpulses = new Array(b2Settings.b2_maxManifoldPoints)
};
b2ContactImpulse.prototype.normalImpulses = new Array(b2Settings.b2_maxManifoldPoints);
b2ContactImpulse.prototype.tangentImpulses = new Array(b2Settings.b2_maxManifoldPoints);var b2DistanceJointDef = function() {
  b2JointDef.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2DistanceJointDef.prototype, b2JointDef.prototype);
b2DistanceJointDef.prototype._super = function() {
  b2JointDef.prototype.__constructor.apply(this, arguments)
};
b2DistanceJointDef.prototype.__constructor = function() {
  this.type = b2Joint.e_distanceJoint;
  this.length = 1;
  this.frequencyHz = 0;
  this.dampingRatio = 0
};
b2DistanceJointDef.prototype.__varz = function() {
  this.localAnchorA = new b2Vec2;
  this.localAnchorB = new b2Vec2
};
b2DistanceJointDef.prototype.localAnchorA = new b2Vec2;
b2DistanceJointDef.prototype.localAnchorB = new b2Vec2;
b2DistanceJointDef.prototype.length = null;
b2DistanceJointDef.prototype.frequencyHz = null;
b2DistanceJointDef.prototype.dampingRatio = null;
b2DistanceJointDef.prototype.Initialize = function(bA, bB, anchorA, anchorB) {
  this.bodyA = bA;
  this.bodyB = bB;
  this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchorA));
  this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchorB));
  var dX = anchorB.x - anchorA.x;
  var dY = anchorB.y - anchorA.y;
  this.length = Math.sqrt(dX * dX + dY * dY);
  this.frequencyHz = 0;
  this.dampingRatio = 0
};var b2FrictionJointDef = function() {
  b2JointDef.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2FrictionJointDef.prototype, b2JointDef.prototype);
b2FrictionJointDef.prototype._super = function() {
  b2JointDef.prototype.__constructor.apply(this, arguments)
};
b2FrictionJointDef.prototype.__constructor = function() {
  this.type = b2Joint.e_frictionJoint;
  this.maxForce = 0;
  this.maxTorque = 0
};
b2FrictionJointDef.prototype.__varz = function() {
  this.localAnchorA = new b2Vec2;
  this.localAnchorB = new b2Vec2
};
b2FrictionJointDef.prototype.localAnchorA = new b2Vec2;
b2FrictionJointDef.prototype.localAnchorB = new b2Vec2;
b2FrictionJointDef.prototype.maxForce = null;
b2FrictionJointDef.prototype.maxTorque = null;
b2FrictionJointDef.prototype.Initialize = function(bA, bB, anchor) {
  this.bodyA = bA;
  this.bodyB = bB;
  this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
  this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor))
};var b2TensorDampingController = function() {
  b2Controller.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2TensorDampingController.prototype, b2Controller.prototype);
b2TensorDampingController.prototype._super = function() {
  b2Controller.prototype.__constructor.apply(this, arguments)
};
b2TensorDampingController.prototype.__constructor = function() {
};
b2TensorDampingController.prototype.__varz = function() {
  this.T = new b2Mat22
};
b2TensorDampingController.prototype.T = new b2Mat22;
b2TensorDampingController.prototype.maxTimestep = 0;
b2TensorDampingController.prototype.SetAxisAligned = function(xDamping, yDamping) {
  this.T.col1.x = -xDamping;
  this.T.col1.y = 0;
  this.T.col2.x = 0;
  this.T.col2.y = -yDamping;
  if(xDamping > 0 || yDamping > 0) {
    this.maxTimestep = 1 / Math.max(xDamping, yDamping)
  }else {
    this.maxTimestep = 0
  }
};
b2TensorDampingController.prototype.Step = function(step) {
  var timestep = step.dt;
  if(timestep <= Number.MIN_VALUE) {
    return
  }
  if(timestep > this.maxTimestep && this.maxTimestep > 0) {
    timestep = this.maxTimestep
  }
  for(var i = m_bodyList;i;i = i.nextBody) {
    var body = i.body;
    if(!body.IsAwake()) {
      continue
    }
    var damping = body.GetWorldVector(b2Math.MulMV(this.T, body.GetLocalVector(body.GetLinearVelocity())));
    body.SetLinearVelocity(new b2Vec2(body.GetLinearVelocity().x + damping.x * timestep, body.GetLinearVelocity().y + damping.y * timestep))
  }
};var b2WeldJointDef = function() {
  b2JointDef.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2WeldJointDef.prototype, b2JointDef.prototype);
b2WeldJointDef.prototype._super = function() {
  b2JointDef.prototype.__constructor.apply(this, arguments)
};
b2WeldJointDef.prototype.__constructor = function() {
  this.type = b2Joint.e_weldJoint;
  this.referenceAngle = 0
};
b2WeldJointDef.prototype.__varz = function() {
  this.localAnchorA = new b2Vec2;
  this.localAnchorB = new b2Vec2
};
b2WeldJointDef.prototype.localAnchorA = new b2Vec2;
b2WeldJointDef.prototype.localAnchorB = new b2Vec2;
b2WeldJointDef.prototype.referenceAngle = null;
b2WeldJointDef.prototype.Initialize = function(bA, bB, anchor) {
  this.bodyA = bA;
  this.bodyB = bB;
  this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
  this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
  this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle()
};var b2GearJointDef = function() {
  b2JointDef.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2GearJointDef.prototype, b2JointDef.prototype);
b2GearJointDef.prototype._super = function() {
  b2JointDef.prototype.__constructor.apply(this, arguments)
};
b2GearJointDef.prototype.__constructor = function() {
  this.type = b2Joint.e_gearJoint;
  this.joint1 = null;
  this.joint2 = null;
  this.ratio = 1
};
b2GearJointDef.prototype.__varz = function() {
};
b2GearJointDef.prototype.joint1 = null;
b2GearJointDef.prototype.joint2 = null;
b2GearJointDef.prototype.ratio = null;var b2PolygonShape = function() {
  b2Shape.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2PolygonShape.prototype, b2Shape.prototype);
b2PolygonShape.prototype._super = function() {
  b2Shape.prototype.__constructor.apply(this, arguments)
};
b2PolygonShape.prototype.__constructor = function() {
  m_type = e_polygonShape;
  m_centroid = new b2Vec2;
  m_vertices = new Array;
  m_normals = new Array
};
b2PolygonShape.prototype.__varz = function() {
};
b2PolygonShape.s_mat = new b2Mat22;
b2PolygonShape.AsArray = function(vertices, vertexCount) {
  var polygonShape = new b2PolygonShape;
  polygonShape.SetAsArray(vertices, vertexCount);
  return polygonShape
};
b2PolygonShape.AsVector = function(vertices, vertexCount) {
  var polygonShape = new b2PolygonShape;
  polygonShape.SetAsVector(vertices, vertexCount);
  return polygonShape
};
b2PolygonShape.AsBox = function(hx, hy) {
  var polygonShape = new b2PolygonShape;
  polygonShape.SetAsBox(hx, hy);
  return polygonShape
};
b2PolygonShape.AsOrientedBox = function(hx, hy, center, angle) {
  var polygonShape = new b2PolygonShape;
  polygonShape.SetAsOrientedBox(hx, hy, center, angle);
  return polygonShape
};
b2PolygonShape.AsEdge = function(v1, v2) {
  var polygonShape = new b2PolygonShape;
  polygonShape.SetAsEdge(v1, v2);
  return polygonShape
};
b2PolygonShape.ComputeCentroid = function(vs, count) {
  var c = new b2Vec2;
  var area = 0;
  var p1X = 0;
  var p1Y = 0;
  var inv3 = 1 / 3;
  for(var i = 0;i < count;++i) {
    var p2 = vs[i];
    var p3 = i + 1 < count ? vs[parseInt(i + 1)] : vs[0];
    var e1X = p2.x - p1X;
    var e1Y = p2.y - p1Y;
    var e2X = p3.x - p1X;
    var e2Y = p3.y - p1Y;
    var D = e1X * e2Y - e1Y * e2X;
    var triangleArea = 0.5 * D;
    area += triangleArea;
    c.x += triangleArea * inv3 * (p1X + p2.x + p3.x);
    c.y += triangleArea * inv3 * (p1Y + p2.y + p3.y)
  }
  c.x *= 1 / area;
  c.y *= 1 / area;
  return c
};
b2PolygonShape.prototype.Validate = function() {
  return false
};
b2PolygonShape.prototype.Reserve = function(count) {
  for(var i = m_vertices.length;i < count;i++) {
    m_vertices[i] = new b2Vec2;
    m_normals[i] = new b2Vec2
  }
};
b2PolygonShape.prototype.Copy = function() {
  var s = new b2PolygonShape;
  s.Set(this);
  return s
};
b2PolygonShape.prototype.Set = function(other) {
  this._super.Set(other);
  if(typeof other === "b2PolygonShape") {
    var other2 = other;
    m_centroid.SetV(other2.m_centroid);
    m_vertexCount = other2.m_vertexCount;
    this.Reserve(m_vertexCount);
    for(var i = 0;i < m_vertexCount;i++) {
      m_vertices[i].SetV(other2.m_vertices[i]);
      m_normals[i].SetV(other2.m_normals[i])
    }
  }
};
b2PolygonShape.prototype.SetAsArray = function(vertices, vertexCount) {
  var v = new Array;
  for(var tVec in vertices) {
    v.push(tVec)
  }
  this.SetAsVector(v, vertexCount)
};
b2PolygonShape.prototype.SetAsVector = function(vertices, vertexCount) {
  if(vertexCount == 0) {
    vertexCount = vertices.length
  }
  b2Settings.b2Assert(2 <= vertexCount);
  m_vertexCount = vertexCount;
  this.Reserve(vertexCount);
  var i = 0;
  for(i = 0;i < m_vertexCount;i++) {
    m_vertices[i].SetV(vertices[i])
  }
  for(i = 0;i < m_vertexCount;++i) {
    var i1 = i;
    var i2 = i + 1 < m_vertexCount ? i + 1 : 0;
    var edge = b2Math.SubtractVV(m_vertices[i2], m_vertices[i1]);
    b2Settings.b2Assert(edge.LengthSquared() > Number.MIN_VALUE);
    m_normals[i].SetV(b2Math.CrossVF(edge, 1));
    m_normals[i].Normalize()
  }
  m_centroid = b2PolygonShape.ComputeCentroid(m_vertices, m_vertexCount)
};
b2PolygonShape.prototype.SetAsBox = function(hx, hy) {
  m_vertexCount = 4;
  this.Reserve(4);
  m_vertices[0].Set(-hx, -hy);
  m_vertices[1].Set(hx, -hy);
  m_vertices[2].Set(hx, hy);
  m_vertices[3].Set(-hx, hy);
  m_normals[0].Set(0, -1);
  m_normals[1].Set(1, 0);
  m_normals[2].Set(0, 1);
  m_normals[3].Set(-1, 0);
  m_centroid.SetZero()
};
b2PolygonShape.prototype.SetAsOrientedBox = function(hx, hy, center, angle) {
  m_vertexCount = 4;
  this.Reserve(4);
  m_vertices[0].Set(-hx, -hy);
  m_vertices[1].Set(hx, -hy);
  m_vertices[2].Set(hx, hy);
  m_vertices[3].Set(-hx, hy);
  m_normals[0].Set(0, -1);
  m_normals[1].Set(1, 0);
  m_normals[2].Set(0, 1);
  m_normals[3].Set(-1, 0);
  m_centroid = center;
  var xf = new b2Transform;
  xf.position = center;
  xf.R.Set(angle);
  for(var i = 0;i < m_vertexCount;++i) {
    m_vertices[i] = b2Math.MulX(xf, m_vertices[i]);
    m_normals[i] = b2Math.MulMV(xf.R, m_normals[i])
  }
};
b2PolygonShape.prototype.SetAsEdge = function(v1, v2) {
  m_vertexCount = 2;
  this.Reserve(2);
  m_vertices[0].SetV(v1);
  m_vertices[1].SetV(v2);
  m_centroid.x = 0.5 * (v1.x + v2.x);
  m_centroid.y = 0.5 * (v1.y + v2.y);
  m_normals[0] = b2Math.CrossVF(b2Math.SubtractVV(v2, v1), 1);
  m_normals[0].Normalize();
  m_normals[1].x = -m_normals[0].x;
  m_normals[1].y = -m_normals[0].y
};
b2PolygonShape.prototype.TestPoint = function(xf, p) {
  var tVec;
  var tMat = xf.R;
  var tX = p.x - xf.position.x;
  var tY = p.y - xf.position.y;
  var pLocalX = tX * tMat.col1.x + tY * tMat.col1.y;
  var pLocalY = tX * tMat.col2.x + tY * tMat.col2.y;
  for(var i = 0;i < m_vertexCount;++i) {
    tVec = m_vertices[i];
    tX = pLocalX - tVec.x;
    tY = pLocalY - tVec.y;
    tVec = m_normals[i];
    var dot = tVec.x * tX + tVec.y * tY;
    if(dot > 0) {
      return false
    }
  }
  return true
};
b2PolygonShape.prototype.RayCast = function(output, input, transform) {
  var lower = 0;
  var upper = input.maxFraction;
  var tX;
  var tY;
  var tMat;
  var tVec;
  tX = input.p1.x - transform.position.x;
  tY = input.p1.y - transform.position.y;
  tMat = transform.R;
  var p1X = tX * tMat.col1.x + tY * tMat.col1.y;
  var p1Y = tX * tMat.col2.x + tY * tMat.col2.y;
  tX = input.p2.x - transform.position.x;
  tY = input.p2.y - transform.position.y;
  tMat = transform.R;
  var p2X = tX * tMat.col1.x + tY * tMat.col1.y;
  var p2Y = tX * tMat.col2.x + tY * tMat.col2.y;
  var dX = p2X - p1X;
  var dY = p2Y - p1Y;
  var index = -1;
  for(var i = 0;i < m_vertexCount;++i) {
    tVec = m_vertices[i];
    tX = tVec.x - p1X;
    tY = tVec.y - p1Y;
    tVec = m_normals[i];
    var numerator = tVec.x * tX + tVec.y * tY;
    var denominator = tVec.x * dX + tVec.y * dY;
    if(denominator == 0) {
      if(numerator < 0) {
        return false
      }
    }else {
      if(denominator < 0 && numerator < lower * denominator) {
        lower = numerator / denominator;
        index = i
      }else {
        if(denominator > 0 && numerator < upper * denominator) {
          upper = numerator / denominator
        }
      }
    }
    if(upper < lower - Number.MIN_VALUE) {
      return false
    }
  }
  if(index >= 0) {
    output.fraction = lower;
    tMat = transform.R;
    tVec = m_normals[index];
    output.normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
    output.normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
    return true
  }
  return false
};
b2PolygonShape.prototype.ComputeAABB = function(aabb, xf) {
  var tMat = xf.R;
  var tVec = m_vertices[0];
  var lowerX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
  var lowerY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
  var upperX = lowerX;
  var upperY = lowerY;
  for(var i = 1;i < m_vertexCount;++i) {
    tVec = m_vertices[i];
    var vX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    var vY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    lowerX = lowerX < vX ? lowerX : vX;
    lowerY = lowerY < vY ? lowerY : vY;
    upperX = upperX > vX ? upperX : vX;
    upperY = upperY > vY ? upperY : vY
  }
  aabb.lowerBound.x = lowerX - m_radius;
  aabb.lowerBound.y = lowerY - m_radius;
  aabb.upperBound.x = upperX + m_radius;
  aabb.upperBound.y = upperY + m_radius
};
b2PolygonShape.prototype.ComputeMass = function(massData, density) {
  if(m_vertexCount == 2) {
    massData.center.x = 0.5 * (m_vertices[0].x + m_vertices[1].x);
    massData.center.y = 0.5 * (m_vertices[0].y + m_vertices[1].y);
    massData.mass = 0;
    massData.I = 0;
    return
  }
  var centerX = 0;
  var centerY = 0;
  var area = 0;
  var I = 0;
  var p1X = 0;
  var p1Y = 0;
  var k_inv3 = 1 / 3;
  for(var i = 0;i < m_vertexCount;++i) {
    var p2 = m_vertices[i];
    var p3 = i + 1 < m_vertexCount ? m_vertices[parseInt(i + 1)] : m_vertices[0];
    var e1X = p2.x - p1X;
    var e1Y = p2.y - p1Y;
    var e2X = p3.x - p1X;
    var e2Y = p3.y - p1Y;
    var D = e1X * e2Y - e1Y * e2X;
    var triangleArea = 0.5 * D;
    area += triangleArea;
    centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x);
    centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y);
    var px = p1X;
    var py = p1Y;
    var ex1 = e1X;
    var ey1 = e1Y;
    var ex2 = e2X;
    var ey2 = e2Y;
    var intx2 = k_inv3 * (0.25 * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px * ex2)) + 0.5 * px * px;
    var inty2 = k_inv3 * (0.25 * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py * ey2)) + 0.5 * py * py;
    I += D * (intx2 + inty2)
  }
  massData.mass = density * area;
  centerX *= 1 / area;
  centerY *= 1 / area;
  massData.center.Set(centerX, centerY);
  massData.I = density * I
};
b2PolygonShape.prototype.ComputeSubmergedArea = function(normal, offset, xf, c) {
  var normalL = b2Math.MulTMV(xf.R, normal);
  var offsetL = offset - b2Math.Dot(normal, xf.position);
  var depths = new Array;
  var diveCount = 0;
  var intoIndex = -1;
  var outoIndex = -1;
  var lastSubmerged = false;
  var i = 0;
  for(i = 0;i < m_vertexCount;++i) {
    depths[i] = b2Math.Dot(normalL, m_vertices[i]) - offsetL;
    var isSubmerged = depths[i] < -Number.MIN_VALUE;
    if(i > 0) {
      if(isSubmerged) {
        if(!lastSubmerged) {
          intoIndex = i - 1;
          diveCount++
        }
      }else {
        if(lastSubmerged) {
          outoIndex = i - 1;
          diveCount++
        }
      }
    }
    lastSubmerged = isSubmerged
  }
  switch(diveCount) {
    case 0:
      if(lastSubmerged) {
        var md = new b2MassData;
        this.ComputeMass(md, 1);
        c.SetV(b2Math.MulX(xf, md.center));
        return md.mass
      }else {
        return 0
      }
      break;
    case 1:
      if(intoIndex == -1) {
        intoIndex = m_vertexCount - 1
      }else {
        outoIndex = m_vertexCount - 1
      }
      break
  }
  var intoIndex2 = (intoIndex + 1) % m_vertexCount;
  var outoIndex2 = (outoIndex + 1) % m_vertexCount;
  var intoLamdda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
  var outoLamdda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);
  var intoVec = new b2Vec2(m_vertices[intoIndex].x * (1 - intoLamdda) + m_vertices[intoIndex2].x * intoLamdda, m_vertices[intoIndex].y * (1 - intoLamdda) + m_vertices[intoIndex2].y * intoLamdda);
  var outoVec = new b2Vec2(m_vertices[outoIndex].x * (1 - outoLamdda) + m_vertices[outoIndex2].x * outoLamdda, m_vertices[outoIndex].y * (1 - outoLamdda) + m_vertices[outoIndex2].y * outoLamdda);
  var area = 0;
  var center = new b2Vec2;
  var p2 = m_vertices[intoIndex2];
  var p3;
  i = intoIndex2;
  while(i != outoIndex2) {
    i = (i + 1) % m_vertexCount;
    if(i == outoIndex2) {
      p3 = outoVec
    }else {
      p3 = m_vertices[i]
    }
    var triangleArea = 0.5 * ((p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x));
    area += triangleArea;
    center.x += triangleArea * (intoVec.x + p2.x + p3.x) / 3;
    center.y += triangleArea * (intoVec.y + p2.y + p3.y) / 3;
    p2 = p3
  }
  center.Multiply(1 / area);
  c.SetV(b2Math.MulX(xf, center));
  return area
};
b2PolygonShape.prototype.GetVertexCount = function() {
  return m_vertexCount
};
b2PolygonShape.prototype.GetVertices = function() {
  return m_vertices
};
b2PolygonShape.prototype.GetNormals = function() {
  return m_normals
};
b2PolygonShape.prototype.GetSupport = function(d) {
  var bestIndex = 0;
  var bestValue = m_vertices[0].x * d.x + m_vertices[0].y * d.y;
  for(var i = 1;i < m_vertexCount;++i) {
    var value = m_vertices[i].x * d.x + m_vertices[i].y * d.y;
    if(value > bestValue) {
      bestIndex = i;
      bestValue = value
    }
  }
  return bestIndex
};
b2PolygonShape.prototype.GetSupportVertex = function(d) {
  var bestIndex = 0;
  var bestValue = m_vertices[0].x * d.x + m_vertices[0].y * d.y;
  for(var i = 1;i < m_vertexCount;++i) {
    var value = m_vertices[i].x * d.x + m_vertices[i].y * d.y;
    if(value > bestValue) {
      bestIndex = i;
      bestValue = value
    }
  }
  return m_vertices[bestIndex]
};var b2FrictionJoint = function() {
  b2Joint.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2FrictionJoint.prototype, b2Joint.prototype);
b2FrictionJoint.prototype._super = function() {
  b2Joint.prototype.__constructor.apply(this, arguments)
};
b2FrictionJoint.prototype.__constructor = function(def) {
  this._super(def);
  this.m_localAnchorA.SetV(def.localAnchorA);
  this.m_localAnchorB.SetV(def.localAnchorB);
  this.m_linearMass.SetZero();
  this.m_angularMass = 0;
  this.m_linearImpulse.SetZero();
  this.m_angularImpulse = 0;
  this.m_maxForce = def.maxForce;
  this.m_maxTorque = def.maxTorque
};
b2FrictionJoint.prototype.__varz = function() {
  this.m_localAnchorA = new b2Vec2;
  this.m_localAnchorB = new b2Vec2;
  this.m_linearImpulse = new b2Vec2;
  this.m_linearMass = new b2Mat22
};
b2FrictionJoint.prototype.m_localAnchorA = new b2Vec2;
b2FrictionJoint.prototype.m_localAnchorB = new b2Vec2;
b2FrictionJoint.prototype.m_linearImpulse = new b2Vec2;
b2FrictionJoint.prototype.m_angularImpulse = null;
b2FrictionJoint.prototype.m_maxForce = null;
b2FrictionJoint.prototype.m_maxTorque = null;
b2FrictionJoint.prototype.m_linearMass = new b2Mat22;
b2FrictionJoint.prototype.m_angularMass = null;
b2FrictionJoint.prototype.GetAnchorA = function() {
  return m_bodyA.GetWorldPoint(this.m_localAnchorA)
};
b2FrictionJoint.prototype.GetAnchorB = function() {
  return m_bodyB.GetWorldPoint(this.m_localAnchorB)
};
b2FrictionJoint.prototype.GetReactionForce = function(inv_dt) {
  return new b2Vec2(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y)
};
b2FrictionJoint.prototype.GetReactionTorque = function(inv_dt) {
  return inv_dt * this.m_angularImpulse
};
b2FrictionJoint.prototype.SetMaxForce = function(force) {
  this.m_maxForce = force
};
b2FrictionJoint.prototype.GetMaxForce = function() {
  return this.m_maxForce
};
b2FrictionJoint.prototype.SetMaxTorque = function(torque) {
  this.m_maxTorque = torque
};
b2FrictionJoint.prototype.GetMaxTorque = function() {
  return this.m_maxTorque
};var b2Fixture = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Fixture.prototype.__constructor = function() {
  m_aabb = new b2AABB;
  m_userData = null;
  m_body = null;
  m_next = null;
  m_shape = null;
  m_density = 0;
  m_friction = 0;
  m_restitution = 0
};
b2Fixture.prototype.__varz = function() {
};
b2Fixture.prototype.m_massData = null;
b2Fixture.prototype.GetType = function() {
  return m_shape.GetType()
};
b2Fixture.prototype.GetShape = function() {
  return m_shape
};
b2Fixture.prototype.SetSensor = function(sensor) {
  if(m_isSensor == sensor) {
    return
  }
  m_isSensor = sensor;
  if(m_body == null) {
    return
  }
  var edge = m_body.GetContactList();
  while(edge) {
    var contact = edge.contact;
    var fixtureA = contact.GetFixtureA();
    var fixtureB = contact.GetFixtureB();
    if(fixtureA == this || fixtureB == this) {
      contact.SetSensor(fixtureA.IsSensor() || fixtureB.IsSensor())
    }
    edge = edge.next
  }
};
b2Fixture.prototype.IsSensor = function() {
  return m_isSensor
};
b2Fixture.prototype.SetFilterData = function(filter) {
  m_filter = filter.Copy();
  if(m_body) {
    return
  }
  var edge = m_body.GetContactList();
  while(edge) {
    var contact = edge.contact;
    var fixtureA = contact.GetFixtureA();
    var fixtureB = contact.GetFixtureB();
    if(fixtureA == this || fixtureB == this) {
      contact.FlagForFiltering()
    }
    edge = edge.next
  }
};
b2Fixture.prototype.GetFilterData = function() {
  return m_filter.Copy()
};
b2Fixture.prototype.GetBody = function() {
  return m_body
};
b2Fixture.prototype.GetNext = function() {
  return m_next
};
b2Fixture.prototype.GetUserData = function() {
  return m_userData
};
b2Fixture.prototype.SetUserData = function(data) {
  m_userData = data
};
b2Fixture.prototype.TestPoint = function(p) {
  return m_shape.TestPoint(m_body.GetTransform(), p)
};
b2Fixture.prototype.RayCast = function(output, input) {
  return m_shape.RayCast(output, input, m_body.GetTransform())
};
b2Fixture.prototype.GetMassData = function(massData) {
  if(massData == null) {
    massData = new b2MassData
  }
  m_shape.ComputeMass(massData, m_density);
  return massData
};
b2Fixture.prototype.SetDensity = function(density) {
  m_density = density
};
b2Fixture.prototype.GetDensity = function() {
  return m_density
};
b2Fixture.prototype.GetFriction = function() {
  return m_friction
};
b2Fixture.prototype.SetFriction = function(friction) {
  m_friction = friction
};
b2Fixture.prototype.GetRestitution = function() {
  return m_restitution
};
b2Fixture.prototype.SetRestitution = function(restitution) {
  m_restitution = restitution
};
b2Fixture.prototype.GetAABB = function() {
  return m_aabb
};var b2DynamicTreeNode = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2DynamicTreeNode.prototype.__constructor = function() {
};
b2DynamicTreeNode.prototype.__varz = function() {
  this.aabb = new b2AABB
};
b2DynamicTreeNode.prototype.userData = null;
b2DynamicTreeNode.prototype.aabb = new b2AABB;
b2DynamicTreeNode.prototype.parent = null;
b2DynamicTreeNode.prototype.child1 = null;
b2DynamicTreeNode.prototype.child2 = null;
b2DynamicTreeNode.prototype.IsLeaf = function() {
  return this.child1 == null
};var b2MouseJoint = function() {
  b2Joint.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2MouseJoint.prototype, b2Joint.prototype);
b2MouseJoint.prototype._super = function() {
  b2Joint.prototype.__constructor.apply(this, arguments)
};
b2MouseJoint.prototype.__constructor = function(def) {
  this._super(def);
  this.m_target.SetV(def.target);
  var tX = this.m_target.x - m_bodyB.m_xf.position.x;
  var tY = this.m_target.y - m_bodyB.m_xf.position.y;
  var tMat = m_bodyB.m_xf.R;
  this.m_localAnchor.x = tX * tMat.col1.x + tY * tMat.col1.y;
  this.m_localAnchor.y = tX * tMat.col2.x + tY * tMat.col2.y;
  this.m_maxForce = def.maxForce;
  this.m_impulse.SetZero();
  this.m_frequencyHz = def.frequencyHz;
  this.m_dampingRatio = def.dampingRatio;
  this.m_beta = 0;
  this.m_gamma = 0
};
b2MouseJoint.prototype.__varz = function() {
  this.K = new b2Mat22;
  this.K1 = new b2Mat22;
  this.K2 = new b2Mat22;
  this.m_localAnchor = new b2Vec2;
  this.m_target = new b2Vec2;
  this.m_impulse = new b2Vec2;
  this.m_mass = new b2Mat22;
  this.m_C = new b2Vec2
};
b2MouseJoint.prototype.K = new b2Mat22;
b2MouseJoint.prototype.K1 = new b2Mat22;
b2MouseJoint.prototype.K2 = new b2Mat22;
b2MouseJoint.prototype.m_localAnchor = new b2Vec2;
b2MouseJoint.prototype.m_target = new b2Vec2;
b2MouseJoint.prototype.m_impulse = new b2Vec2;
b2MouseJoint.prototype.m_mass = new b2Mat22;
b2MouseJoint.prototype.m_C = new b2Vec2;
b2MouseJoint.prototype.m_maxForce = null;
b2MouseJoint.prototype.m_frequencyHz = null;
b2MouseJoint.prototype.m_dampingRatio = null;
b2MouseJoint.prototype.m_beta = null;
b2MouseJoint.prototype.m_gamma = null;
b2MouseJoint.prototype.GetAnchorA = function() {
  return this.m_target
};
b2MouseJoint.prototype.GetAnchorB = function() {
  return m_bodyB.GetWorldPoint(this.m_localAnchor)
};
b2MouseJoint.prototype.GetReactionForce = function(inv_dt) {
  return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y)
};
b2MouseJoint.prototype.GetReactionTorque = function(inv_dt) {
  return 0
};
b2MouseJoint.prototype.GetTarget = function() {
  return this.m_target
};
b2MouseJoint.prototype.SetTarget = function(target) {
  if(m_bodyB.IsAwake() == false) {
    m_bodyB.SetAwake(true)
  }
  this.m_target = target
};
b2MouseJoint.prototype.GetMaxForce = function() {
  return this.m_maxForce
};
b2MouseJoint.prototype.SetMaxForce = function(maxForce) {
  this.m_maxForce = maxForce
};
b2MouseJoint.prototype.GetFrequency = function() {
  return this.m_frequencyHz
};
b2MouseJoint.prototype.SetFrequency = function(hz) {
  this.m_frequencyHz = hz
};
b2MouseJoint.prototype.GetDampingRatio = function() {
  return this.m_dampingRatio
};
b2MouseJoint.prototype.SetDampingRatio = function(ratio) {
  this.m_dampingRatio = ratio
};var b2BodyDef = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2BodyDef.prototype.__constructor = function() {
  this.userData = null;
  this.position.Set(0, 0);
  this.angle = 0;
  this.linearVelocity.Set(0, 0);
  this.angularVelocity = 0;
  this.linearDamping = 0;
  this.angularDamping = 0;
  this.allowSleep = true;
  this.awake = true;
  this.fixedRotation = false;
  this.bullet = false;
  this.type = b2Body.b2_staticBody;
  this.active = true;
  this.inertiaScale = 1
};
b2BodyDef.prototype.__varz = function() {
  this.position = new b2Vec2;
  this.linearVelocity = new b2Vec2
};
b2BodyDef.prototype.type = 0;
b2BodyDef.prototype.position = new b2Vec2;
b2BodyDef.prototype.angle = null;
b2BodyDef.prototype.linearVelocity = new b2Vec2;
b2BodyDef.prototype.angularVelocity = null;
b2BodyDef.prototype.linearDamping = null;
b2BodyDef.prototype.angularDamping = null;
b2BodyDef.prototype.allowSleep = null;
b2BodyDef.prototype.awake = null;
b2BodyDef.prototype.fixedRotation = null;
b2BodyDef.prototype.bullet = null;
b2BodyDef.prototype.active = null;
b2BodyDef.prototype.userData = null;
b2BodyDef.prototype.inertiaScale = null;var b2DynamicTreeBroadPhase = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2DynamicTreeBroadPhase.prototype.__constructor = function() {
};
b2DynamicTreeBroadPhase.prototype.__varz = function() {
  this.m_tree = new b2DynamicTree;
  this.m_moveBuffer = new Array;
  this.m_pairBuffer = new Array
};
b2DynamicTreeBroadPhase.prototype.m_tree = new b2DynamicTree;
b2DynamicTreeBroadPhase.prototype.m_proxyCount = 0;
b2DynamicTreeBroadPhase.prototype.m_moveBuffer = new Array;
b2DynamicTreeBroadPhase.prototype.m_pairBuffer = new Array;
b2DynamicTreeBroadPhase.prototype.m_pairCount = 0;
b2DynamicTreeBroadPhase.prototype.BufferMove = function(proxy) {
  this.m_moveBuffer[m_moveBuffer.length] = proxy
};
b2DynamicTreeBroadPhase.prototype.UnBufferMove = function(proxy) {
  var i = this.m_moveBuffer.indexOf(proxy);
  this.m_moveBuffer.splice(i, 1)
};
b2DynamicTreeBroadPhase.prototype.ComparePairs = function(pair1, pair2) {
  return 0
};
b2DynamicTreeBroadPhase.prototype.CreateProxy = function(aabb, userData) {
  var proxy = this.m_tree.CreateProxy(aabb, userData);
  ++this.m_proxyCount;
  this.BufferMove(proxy);
  return proxy
};
b2DynamicTreeBroadPhase.prototype.DestroyProxy = function(proxy) {
  this.UnBufferMove(proxy);
  --this.m_proxyCount;
  this.m_tree.DestroyProxy(proxy)
};
b2DynamicTreeBroadPhase.prototype.MoveProxy = function(proxy, aabb, displacement) {
  var buffer = this.m_tree.MoveProxy(proxy, aabb, displacement);
  if(buffer) {
    this.BufferMove(proxy)
  }
};
b2DynamicTreeBroadPhase.prototype.TestOverlap = function(proxyA, proxyB) {
  var aabbA = this.m_tree.GetFatAABB(proxyA);
  var aabbB = this.m_tree.GetFatAABB(proxyB);
  return aabbA.TestOverlap(aabbB)
};
b2DynamicTreeBroadPhase.prototype.GetUserData = function(proxy) {
  return this.m_tree.GetUserData(proxy)
};
b2DynamicTreeBroadPhase.prototype.GetFatAABB = function(proxy) {
  return this.m_tree.GetFatAABB(proxy)
};
b2DynamicTreeBroadPhase.prototype.GetProxyCount = function() {
  return this.m_proxyCount
};
b2DynamicTreeBroadPhase.prototype.UpdatePairs = function(callback) {
  this.m_pairCount = 0;
  for(var queryProxy in this.m_moveBuffer) {
    function QueryCallback(proxy) {
      if(proxy == queryProxy) {
        return true
      }
      if(this.m_pairCount == this.m_pairBuffer.length) {
        this.m_pairBuffer[this.m_pairCount] = new b2DynamicTreePair
      }
      var pair = this.m_pairBuffer[this.m_pairCount];
      pair.proxyA = proxy < queryProxy ? proxy : queryProxy;
      pair.proxyB = proxy >= queryProxy ? proxy : queryProxy;
      ++this.m_pairCount;
      return true
    }
    var fatAABB = this.m_tree.GetFatAABB(queryProxy);
    this.m_tree.Query(QueryCallback, fatAABB)
  }
  this.m_moveBuffer.length = 0;
  for(var i = 0;i < this.m_pairCount;) {
    var primaryPair = this.m_pairBuffer[i];
    var userDataA = this.m_tree.GetUserData(primaryPair.proxyA);
    var userDataB = this.m_tree.GetUserData(primaryPair.proxyB);
    callback(userDataA, userDataB);
    ++i;
    while(i < this.m_pairCount) {
      var pair = this.m_pairBuffer[i];
      if(pair.proxyA != primaryPair.proxyA || pair.proxyB != primaryPair.proxyB) {
        break
      }
      ++i
    }
  }
};
b2DynamicTreeBroadPhase.prototype.Query = function(callback, aabb) {
  this.m_tree.Query(callback, aabb)
};
b2DynamicTreeBroadPhase.prototype.RayCast = function(callback, input) {
  this.m_tree.RayCast(callback, input)
};
b2DynamicTreeBroadPhase.prototype.Validate = function() {
};
b2DynamicTreeBroadPhase.prototype.Rebalance = function(iterations) {
  this.m_tree.Rebalance(iterations)
};var b2PrismaticJointDef = function() {
  b2JointDef.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2PrismaticJointDef.prototype, b2JointDef.prototype);
b2PrismaticJointDef.prototype._super = function() {
  b2JointDef.prototype.__constructor.apply(this, arguments)
};
b2PrismaticJointDef.prototype.__constructor = function() {
  this.type = b2Joint.e_prismaticJoint;
  this.localAxisA.Set(1, 0);
  this.referenceAngle = 0;
  this.enableLimit = false;
  this.lowerTranslation = 0;
  this.upperTranslation = 0;
  this.enableMotor = false;
  this.maxMotorForce = 0;
  this.motorSpeed = 0
};
b2PrismaticJointDef.prototype.__varz = function() {
  this.localAnchorA = new b2Vec2;
  this.localAnchorB = new b2Vec2;
  this.localAxisA = new b2Vec2
};
b2PrismaticJointDef.prototype.localAnchorA = new b2Vec2;
b2PrismaticJointDef.prototype.localAnchorB = new b2Vec2;
b2PrismaticJointDef.prototype.localAxisA = new b2Vec2;
b2PrismaticJointDef.prototype.referenceAngle = null;
b2PrismaticJointDef.prototype.enableLimit = null;
b2PrismaticJointDef.prototype.lowerTranslation = null;
b2PrismaticJointDef.prototype.upperTranslation = null;
b2PrismaticJointDef.prototype.enableMotor = null;
b2PrismaticJointDef.prototype.maxMotorForce = null;
b2PrismaticJointDef.prototype.motorSpeed = null;
b2PrismaticJointDef.prototype.Initialize = function(bA, bB, anchor, axis) {
  this.bodyA = bA;
  this.bodyB = bB;
  this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
  this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
  this.localAxisA = this.bodyA.GetLocalVector(axis);
  this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle()
};var b2CircleShape = function() {
  b2Shape.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2CircleShape.prototype, b2Shape.prototype);
b2CircleShape.prototype._super = function() {
  b2Shape.prototype.__constructor.apply(this, arguments)
};
b2CircleShape.prototype.__constructor = function(radius) {
  this._super();
  m_type = e_circleShape;
  m_radius = radius
};
b2CircleShape.prototype.__varz = function() {
};
b2CircleShape.prototype.Copy = function() {
  var s = new b2CircleShape;
  s.Set(this);
  return s
};
b2CircleShape.prototype.Set = function(other) {
  this._super.Set(other);
  if(typeof other === "b2CircleShape") {
    var other2 = other;
    m_p.SetV(other2.m_p)
  }
};
b2CircleShape.prototype.TestPoint = function(transform, p) {
  var tMat = transform.R;
  var dX = transform.position.x + (tMat.col1.x * m_p.x + tMat.col2.x * m_p.y);
  var dY = transform.position.y + (tMat.col1.y * m_p.x + tMat.col2.y * m_p.y);
  dX = p.x - dX;
  dY = p.y - dY;
  return dX * dX + dY * dY <= m_radius * m_radius
};
b2CircleShape.prototype.RayCast = function(output, input, transform) {
  var tMat = transform.R;
  var positionX = transform.position.x + (tMat.col1.x * m_p.x + tMat.col2.x * m_p.y);
  var positionY = transform.position.y + (tMat.col1.y * m_p.x + tMat.col2.y * m_p.y);
  var sX = input.p1.x - positionX;
  var sY = input.p1.y - positionY;
  var b = sX * sX + sY * sY - m_radius * m_radius;
  var rX = input.p2.x - input.p1.x;
  var rY = input.p2.y - input.p1.y;
  var c = sX * rX + sY * rY;
  var rr = rX * rX + rY * rY;
  var sigma = c * c - rr * b;
  if(sigma < 0 || rr < Number.MIN_VALUE) {
    return false
  }
  var a = -(c + Math.sqrt(sigma));
  if(0 <= a && a <= input.maxFraction * rr) {
    a /= rr;
    output.fraction = a;
    output.normal.x = sX + a * rX;
    output.normal.y = sY + a * rY;
    output.normal.Normalize();
    return true
  }
  return false
};
b2CircleShape.prototype.ComputeAABB = function(aabb, transform) {
  var tMat = transform.R;
  var pX = transform.position.x + (tMat.col1.x * m_p.x + tMat.col2.x * m_p.y);
  var pY = transform.position.y + (tMat.col1.y * m_p.x + tMat.col2.y * m_p.y);
  aabb.lowerBound.Set(pX - m_radius, pY - m_radius);
  aabb.upperBound.Set(pX + m_radius, pY + m_radius)
};
b2CircleShape.prototype.ComputeMass = function(massData, density) {
  massData.mass = density * b2Settings.b2_pi * m_radius * m_radius;
  massData.center.SetV(m_p);
  massData.I = massData.mass * (0.5 * m_radius * m_radius + (m_p.x * m_p.x + m_p.y * m_p.y))
};
b2CircleShape.prototype.ComputeSubmergedArea = function(normal, offset, xf, c) {
  var p = b2Math.MulX(xf, m_p);
  var l = -(b2Math.Dot(normal, p) - offset);
  if(l < -m_radius + Number.MIN_VALUE) {
    return 0
  }
  if(l > m_radius) {
    c.SetV(p);
    return Math.PI * m_radius * m_radius
  }
  var r2 = m_radius * m_radius;
  var l2 = l * l;
  var area = r2 * (Math.asin(l / m_radius) + Math.PI / 2) + l * Math.sqrt(r2 - l2);
  var com = -2 / 3 * Math.pow(r2 - l2, 1.5) / area;
  c.x = p.x + normal.x * com;
  c.y = p.y + normal.y * com;
  return area
};
b2CircleShape.prototype.GetLocalPosition = function() {
  return m_p
};
b2CircleShape.prototype.SetLocalPosition = function(position) {
  m_p.SetV(position)
};
b2CircleShape.prototype.GetRadius = function() {
  return m_radius
};
b2CircleShape.prototype.SetRadius = function(radius) {
  m_radius = radius
};var b2GearJoint = function() {
  b2Joint.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2GearJoint.prototype, b2Joint.prototype);
b2GearJoint.prototype._super = function() {
  b2Joint.prototype.__constructor.apply(this, arguments)
};
b2GearJoint.prototype.__constructor = function(def) {
  this._super(def);
  var type1 = def.joint1.m_type;
  var type2 = def.joint2.m_type;
  this.m_revolute1 = null;
  this.m_prismatic1 = null;
  this.m_revolute2 = null;
  this.m_prismatic2 = null;
  var coordinate1;
  var coordinate2;
  this.m_ground1 = def.joint1.GetBodyA();
  m_bodyA = def.joint1.GetBodyB();
  if(type1 == b2Joint.e_revoluteJoint) {
    this.m_revolute1 = def.joint1;
    this.m_groundAnchor1.SetV(this.m_revolute1.m_localAnchor1);
    this.m_localAnchor1.SetV(this.m_revolute1.m_localAnchor2);
    coordinate1 = this.m_revolute1.GetJointAngle()
  }else {
    this.m_prismatic1 = def.joint1;
    this.m_groundAnchor1.SetV(this.m_prismatic1.m_localAnchor1);
    this.m_localAnchor1.SetV(this.m_prismatic1.m_localAnchor2);
    coordinate1 = this.m_prismatic1.GetJointTranslation()
  }
  this.m_ground2 = def.joint2.GetBodyA();
  m_bodyB = def.joint2.GetBodyB();
  if(type2 == b2Joint.e_revoluteJoint) {
    this.m_revolute2 = def.joint2;
    this.m_groundAnchor2.SetV(this.m_revolute2.m_localAnchor1);
    this.m_localAnchor2.SetV(this.m_revolute2.m_localAnchor2);
    coordinate2 = this.m_revolute2.GetJointAngle()
  }else {
    this.m_prismatic2 = def.joint2;
    this.m_groundAnchor2.SetV(this.m_prismatic2.m_localAnchor1);
    this.m_localAnchor2.SetV(this.m_prismatic2.m_localAnchor2);
    coordinate2 = this.m_prismatic2.GetJointTranslation()
  }
  this.m_ratio = def.ratio;
  this.m_constant = coordinate1 + this.m_ratio * coordinate2;
  this.m_impulse = 0
};
b2GearJoint.prototype.__varz = function() {
  this.m_groundAnchor1 = new b2Vec2;
  this.m_groundAnchor2 = new b2Vec2;
  this.m_localAnchor1 = new b2Vec2;
  this.m_localAnchor2 = new b2Vec2;
  this.m_J = new b2Jacobian
};
b2GearJoint.prototype.m_ground1 = null;
b2GearJoint.prototype.m_ground2 = null;
b2GearJoint.prototype.m_revolute1 = null;
b2GearJoint.prototype.m_prismatic1 = null;
b2GearJoint.prototype.m_revolute2 = null;
b2GearJoint.prototype.m_prismatic2 = null;
b2GearJoint.prototype.m_groundAnchor1 = new b2Vec2;
b2GearJoint.prototype.m_groundAnchor2 = new b2Vec2;
b2GearJoint.prototype.m_localAnchor1 = new b2Vec2;
b2GearJoint.prototype.m_localAnchor2 = new b2Vec2;
b2GearJoint.prototype.m_J = new b2Jacobian;
b2GearJoint.prototype.m_constant = null;
b2GearJoint.prototype.m_ratio = null;
b2GearJoint.prototype.m_mass = null;
b2GearJoint.prototype.m_impulse = null;
b2GearJoint.prototype.GetAnchorA = function() {
  return m_bodyA.GetWorldPoint(this.m_localAnchor1)
};
b2GearJoint.prototype.GetAnchorB = function() {
  return m_bodyB.GetWorldPoint(this.m_localAnchor2)
};
b2GearJoint.prototype.GetReactionForce = function(inv_dt) {
  return new b2Vec2(inv_dt * this.m_impulse * this.m_J.linearB.x, inv_dt * this.m_impulse * this.m_J.linearB.y)
};
b2GearJoint.prototype.GetReactionTorque = function(inv_dt) {
  var tMat = m_bodyB.m_xf.R;
  var rX = this.m_localAnchor1.x - m_bodyB.m_sweep.localCenter.x;
  var rY = this.m_localAnchor1.y - m_bodyB.m_sweep.localCenter.y;
  var tX = tMat.col1.x * rX + tMat.col2.x * rY;
  rY = tMat.col1.y * rX + tMat.col2.y * rY;
  rX = tX;
  var PX = this.m_impulse * this.m_J.linearB.x;
  var PY = this.m_impulse * this.m_J.linearB.y;
  return inv_dt * (this.m_impulse * this.m_J.angularB - rX * PY + rY * PX)
};
b2GearJoint.prototype.GetRatio = function() {
  return this.m_ratio
};
b2GearJoint.prototype.SetRatio = function(ratio) {
  this.m_ratio = ratio
};var b2LineJoint = function() {
  b2Joint.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2LineJoint.prototype, b2Joint.prototype);
b2LineJoint.prototype._super = function() {
  b2Joint.prototype.__constructor.apply(this, arguments)
};
b2LineJoint.prototype.__constructor = function(def) {
  this._super(def);
  var tMat;
  var tX;
  var tY;
  m_localAnchor1.SetV(def.localAnchorA);
  m_localAnchor2.SetV(def.localAnchorB);
  m_localXAxis1.SetV(def.localAxisA);
  this.m_localYAxis1.x = -m_localXAxis1.y;
  this.m_localYAxis1.y = m_localXAxis1.x;
  this.m_impulse.SetZero();
  this.m_motorMass = 0;
  this.m_motorImpulse = 0;
  this.m_lowerTranslation = def.lowerTranslation;
  this.m_upperTranslation = def.upperTranslation;
  this.m_maxMotorForce = def.maxMotorForce;
  this.m_motorSpeed = def.motorSpeed;
  this.m_enableLimit = def.enableLimit;
  this.m_enableMotor = def.enableMotor;
  this.m_limitState = e_inactiveLimit;
  this.m_axis.SetZero();
  this.m_perp.SetZero()
};
b2LineJoint.prototype.__varz = function() {
  this.m_localYAxis1 = new b2Vec2;
  this.m_axis = new b2Vec2;
  this.m_perp = new b2Vec2;
  this.m_K = new b2Mat22;
  this.m_impulse = new b2Vec2
};
b2LineJoint.prototype.m_localYAxis1 = new b2Vec2;
b2LineJoint.prototype.m_axis = new b2Vec2;
b2LineJoint.prototype.m_perp = new b2Vec2;
b2LineJoint.prototype.m_s1 = null;
b2LineJoint.prototype.m_s2 = null;
b2LineJoint.prototype.m_a1 = null;
b2LineJoint.prototype.m_a2 = null;
b2LineJoint.prototype.m_K = new b2Mat22;
b2LineJoint.prototype.m_impulse = new b2Vec2;
b2LineJoint.prototype.m_motorMass = null;
b2LineJoint.prototype.m_motorImpulse = null;
b2LineJoint.prototype.m_lowerTranslation = null;
b2LineJoint.prototype.m_upperTranslation = null;
b2LineJoint.prototype.m_maxMotorForce = null;
b2LineJoint.prototype.m_motorSpeed = null;
b2LineJoint.prototype.m_enableLimit = null;
b2LineJoint.prototype.m_enableMotor = null;
b2LineJoint.prototype.m_limitState = 0;
b2LineJoint.prototype.GetAnchorA = function() {
  return m_bodyA.GetWorldPoint(m_localAnchor1)
};
b2LineJoint.prototype.GetAnchorB = function() {
  return m_bodyB.GetWorldPoint(m_localAnchor2)
};
b2LineJoint.prototype.GetReactionForce = function(inv_dt) {
  return new b2Vec2(inv_dt * (this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.x), inv_dt * (this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.y))
};
b2LineJoint.prototype.GetReactionTorque = function(inv_dt) {
  return inv_dt * this.m_impulse.y
};
b2LineJoint.prototype.GetJointTranslation = function() {
  var bA = m_bodyA;
  var bB = m_bodyB;
  var tMat;
  var p1 = bA.GetWorldPoint(m_localAnchor1);
  var p2 = bB.GetWorldPoint(m_localAnchor2);
  var dX = p2.x - p1.x;
  var dY = p2.y - p1.y;
  var axis = bA.GetWorldVector(m_localXAxis1);
  var translation = axis.x * dX + axis.y * dY;
  return translation
};
b2LineJoint.prototype.GetJointSpeed = function() {
  var bA = m_bodyA;
  var bB = m_bodyB;
  var tMat;
  tMat = bA.m_xf.R;
  var r1X = m_localAnchor1.x - bA.m_sweep.localCenter.x;
  var r1Y = m_localAnchor1.y - bA.m_sweep.localCenter.y;
  var tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
  r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
  r1X = tX;
  tMat = bB.m_xf.R;
  var r2X = m_localAnchor2.x - bB.m_sweep.localCenter.x;
  var r2Y = m_localAnchor2.y - bB.m_sweep.localCenter.y;
  tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
  r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
  r2X = tX;
  var p1X = bA.m_sweep.c.x + r1X;
  var p1Y = bA.m_sweep.c.y + r1Y;
  var p2X = bB.m_sweep.c.x + r2X;
  var p2Y = bB.m_sweep.c.y + r2Y;
  var dX = p2X - p1X;
  var dY = p2Y - p1Y;
  var axis = bA.GetWorldVector(m_localXAxis1);
  var v1 = bA.m_linearVelocity;
  var v2 = bB.m_linearVelocity;
  var w1 = bA.m_angularVelocity;
  var w2 = bB.m_angularVelocity;
  var speed = dX * -w1 * axis.y + dY * w1 * axis.x + (axis.x * (v2.x + -w2 * r2Y - v1.x - -w1 * r1Y) + axis.y * (v2.y + w2 * r2X - v1.y - w1 * r1X));
  return speed
};
b2LineJoint.prototype.IsLimitEnabled = function() {
  return this.m_enableLimit
};
b2LineJoint.prototype.EnableLimit = function(flag) {
  m_bodyA.SetAwake(true);
  m_bodyB.SetAwake(true);
  this.m_enableLimit = flag
};
b2LineJoint.prototype.GetLowerLimit = function() {
  return this.m_lowerTranslation
};
b2LineJoint.prototype.GetUpperLimit = function() {
  return this.m_upperTranslation
};
b2LineJoint.prototype.SetLimits = function(lower, upper) {
  m_bodyA.SetAwake(true);
  m_bodyB.SetAwake(true);
  this.m_lowerTranslation = lower;
  this.m_upperTranslation = upper
};
b2LineJoint.prototype.IsMotorEnabled = function() {
  return this.m_enableMotor
};
b2LineJoint.prototype.EnableMotor = function(flag) {
  m_bodyA.SetAwake(true);
  m_bodyB.SetAwake(true);
  this.m_enableMotor = flag
};
b2LineJoint.prototype.SetMotorSpeed = function(speed) {
  m_bodyA.SetAwake(true);
  m_bodyB.SetAwake(true);
  this.m_motorSpeed = speed
};
b2LineJoint.prototype.GetMotorSpeed = function() {
  return this.m_motorSpeed
};
b2LineJoint.prototype.SetMaxMotorForce = function(force) {
  m_bodyA.SetAwake(true);
  m_bodyB.SetAwake(true);
  this.m_maxMotorForce = force
};
b2LineJoint.prototype.GetMaxMotorForce = function() {
  return this.m_maxMotorForce
};
b2LineJoint.prototype.GetMotorForce = function() {
  return this.m_motorImpulse
};var b2ContactSolver = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ContactSolver.prototype.__constructor = function() {
};
b2ContactSolver.prototype.__varz = function() {
  this.m_step = new b2TimeStep
};
b2ContactSolver.s_worldManifold = new b2WorldManifold;
b2ContactSolver.s_psm = new b2PositionSolverManifold;
b2ContactSolver.prototype.m_step = new b2TimeStep;
b2ContactSolver.prototype.m_allocator = null;
b2ContactSolver.prototype.m_constraintCount = 0;
b2ContactSolver.prototype.Initialize = function(step, contacts, contactCount, allocator) {
  var contact;
  this.m_step.Set(step);
  this.m_allocator = allocator;
  var i = 0;
  var tVec;
  var tMat;
  this.m_constraintCount = contactCount;
  while(m_constraints.length < this.m_constraintCount) {
    m_constraints[m_constraints.length] = new b2ContactConstraint
  }
  for(i = 0;i < contactCount;++i) {
    contact = contacts[i];
    var fixtureA = contact.m_fixtureA;
    var fixtureB = contact.m_fixtureB;
    var shapeA = fixtureA.m_shape;
    var shapeB = fixtureB.m_shape;
    var radiusA = shapeA.m_radius;
    var radiusB = shapeB.m_radius;
    var bodyA = fixtureA.m_body;
    var bodyB = fixtureB.m_body;
    var manifold = contact.GetManifold();
    var friction = b2Settings.b2MixFriction(fixtureA.GetFriction(), fixtureB.GetFriction());
    var restitution = b2Settings.b2MixRestitution(fixtureA.GetRestitution(), fixtureB.GetRestitution());
    var vAX = bodyA.m_linearVelocity.x;
    var vAY = bodyA.m_linearVelocity.y;
    var vBX = bodyB.m_linearVelocity.x;
    var vBY = bodyB.m_linearVelocity.y;
    var wA = bodyA.m_angularVelocity;
    var wB = bodyB.m_angularVelocity;
    b2Settings.b2Assert(manifold.m_pointCount > 0);
    b2ContactSolver.s_worldManifold.Initialize(manifold, bodyA.m_xf, radiusA, bodyB.m_xf, radiusB);
    var normalX = b2ContactSolver.s_worldManifold.m_normal.x;
    var normalY = b2ContactSolver.s_worldManifold.m_normal.y;
    var cc = m_constraints[i];
    cc.bodyA = bodyA;
    cc.bodyB = bodyB;
    cc.manifold = manifold;
    cc.normal.x = normalX;
    cc.normal.y = normalY;
    cc.pointCount = manifold.m_pointCount;
    cc.friction = friction;
    cc.restitution = restitution;
    cc.localPlaneNormal.x = manifold.m_localPlaneNormal.x;
    cc.localPlaneNormal.y = manifold.m_localPlaneNormal.y;
    cc.localPoint.x = manifold.m_localPoint.x;
    cc.localPoint.y = manifold.m_localPoint.y;
    cc.radius = radiusA + radiusB;
    cc.type = manifold.m_type;
    for(var k = 0;k < cc.pointCount;++k) {
      var cp = manifold.m_points[k];
      var ccp = cc.points[k];
      ccp.normalImpulse = cp.m_normalImpulse;
      ccp.tangentImpulse = cp.m_tangentImpulse;
      ccp.localPoint.SetV(cp.m_localPoint);
      var rAX = ccp.rA.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyA.m_sweep.c.x;
      var rAY = ccp.rA.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyA.m_sweep.c.y;
      var rBX = ccp.rB.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyB.m_sweep.c.x;
      var rBY = ccp.rB.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyB.m_sweep.c.y;
      var rnA = rAX * normalY - rAY * normalX;
      var rnB = rBX * normalY - rBY * normalX;
      rnA *= rnA;
      rnB *= rnB;
      var kNormal = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rnA + bodyB.m_invI * rnB;
      ccp.normalMass = 1 / kNormal;
      var kEqualized = bodyA.m_mass * bodyA.m_invMass + bodyB.m_mass * bodyB.m_invMass;
      kEqualized += bodyA.m_mass * bodyA.m_invI * rnA + bodyB.m_mass * bodyB.m_invI * rnB;
      ccp.equalizedMass = 1 / kEqualized;
      var tangentX = normalY;
      var tangentY = -normalX;
      var rtA = rAX * tangentY - rAY * tangentX;
      var rtB = rBX * tangentY - rBY * tangentX;
      rtA *= rtA;
      rtB *= rtB;
      var kTangent = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rtA + bodyB.m_invI * rtB;
      ccp.tangentMass = 1 / kTangent;
      ccp.velocityBias = 0;
      var tX = vBX + -wB * rBY - vAX - -wA * rAY;
      var tY = vBY + wB * rBX - vAY - wA * rAX;
      var vRel = cc.normal.x * tX + cc.normal.y * tY;
      if(vRel < -b2Settings.b2_velocityThreshold) {
        ccp.velocityBias += -cc.restitution * vRel
      }
    }
    if(cc.pointCount == 2) {
      var ccp1 = cc.points[0];
      var ccp2 = cc.points[1];
      var invMassA = bodyA.m_invMass;
      var invIA = bodyA.m_invI;
      var invMassB = bodyB.m_invMass;
      var invIB = bodyB.m_invI;
      var rn1A = ccp1.rA.x * normalY - ccp1.rA.y * normalX;
      var rn1B = ccp1.rB.x * normalY - ccp1.rB.y * normalX;
      var rn2A = ccp2.rA.x * normalY - ccp2.rA.y * normalX;
      var rn2B = ccp2.rB.x * normalY - ccp2.rB.y * normalX;
      var k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
      var k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
      var k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;
      var k_maxConditionNumber = 100;
      if(k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
        cc.K.col1.Set(k11, k12);
        cc.K.col2.Set(k12, k22);
        cc.K.GetInverse(cc.normalMass)
      }else {
        cc.pointCount = 1
      }
    }
  }
};
b2ContactSolver.prototype.InitVelocityConstraints = function(step) {
  var tVec;
  var tVec2;
  var tMat;
  for(var i = 0;i < this.m_constraintCount;++i) {
    var c = m_constraints[i];
    var bodyA = c.bodyA;
    var bodyB = c.bodyB;
    var invMassA = bodyA.m_invMass;
    var invIA = bodyA.m_invI;
    var invMassB = bodyB.m_invMass;
    var invIB = bodyB.m_invI;
    var normalX = c.normal.x;
    var normalY = c.normal.y;
    var tangentX = normalY;
    var tangentY = -normalX;
    var tX;
    var j = 0;
    var tCount = 0;
    if(step.warmStarting) {
      tCount = c.pointCount;
      for(j = 0;j < tCount;++j) {
        var ccp = c.points[j];
        ccp.normalImpulse *= step.dtRatio;
        ccp.tangentImpulse *= step.dtRatio;
        var PX = ccp.normalImpulse * normalX + ccp.tangentImpulse * tangentX;
        var PY = ccp.normalImpulse * normalY + ccp.tangentImpulse * tangentY;
        bodyA.m_angularVelocity -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
        bodyA.m_linearVelocity.x -= invMassA * PX;
        bodyA.m_linearVelocity.y -= invMassA * PY;
        bodyB.m_angularVelocity += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
        bodyB.m_linearVelocity.x += invMassB * PX;
        bodyB.m_linearVelocity.y += invMassB * PY
      }
    }else {
      tCount = c.pointCount;
      for(j = 0;j < tCount;++j) {
        var ccp2 = c.points[j];
        ccp2.normalImpulse = 0;
        ccp2.tangentImpulse = 0
      }
    }
  }
};
b2ContactSolver.prototype.SolveVelocityConstraints = function() {
  var j = 0;
  var ccp;
  var rAX;
  var rAY;
  var rBX;
  var rBY;
  var dvX;
  var dvY;
  var vn;
  var vt;
  var lambda;
  var maxFriction;
  var newImpulse;
  var PX;
  var PY;
  var dX;
  var dY;
  var P1X;
  var P1Y;
  var P2X;
  var P2Y;
  var tMat;
  var tVec;
  for(var i = 0;i < this.m_constraintCount;++i) {
    var c = m_constraints[i];
    var bodyA = c.bodyA;
    var bodyB = c.bodyB;
    var wA = bodyA.m_angularVelocity;
    var wB = bodyB.m_angularVelocity;
    var vA = bodyA.m_linearVelocity;
    var vB = bodyB.m_linearVelocity;
    var invMassA = bodyA.m_invMass;
    var invIA = bodyA.m_invI;
    var invMassB = bodyB.m_invMass;
    var invIB = bodyB.m_invI;
    var normalX = c.normal.x;
    var normalY = c.normal.y;
    var tangentX = normalY;
    var tangentY = -normalX;
    var friction = c.friction;
    var tX;
    for(j = 0;j < c.pointCount;j++) {
      ccp = c.points[j];
      dvX = vB.x - wB * ccp.rB.y - vA.x + wA * ccp.rA.y;
      dvY = vB.y + wB * ccp.rB.x - vA.y - wA * ccp.rA.x;
      vt = dvX * tangentX + dvY * tangentY;
      lambda = ccp.tangentMass * -vt;
      maxFriction = friction * ccp.normalImpulse;
      newImpulse = b2Math.Clamp(ccp.tangentImpulse + lambda, -maxFriction, maxFriction);
      lambda = newImpulse - ccp.tangentImpulse;
      PX = lambda * tangentX;
      PY = lambda * tangentY;
      vA.x -= invMassA * PX;
      vA.y -= invMassA * PY;
      wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
      vB.x += invMassB * PX;
      vB.y += invMassB * PY;
      wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
      ccp.tangentImpulse = newImpulse
    }
    var tCount = c.pointCount;
    if(c.pointCount == 1) {
      ccp = c.points[0];
      dvX = vB.x + -wB * ccp.rB.y - vA.x - -wA * ccp.rA.y;
      dvY = vB.y + wB * ccp.rB.x - vA.y - wA * ccp.rA.x;
      vn = dvX * normalX + dvY * normalY;
      lambda = -ccp.normalMass * (vn - ccp.velocityBias);
      newImpulse = ccp.normalImpulse + lambda;
      newImpulse = newImpulse > 0 ? newImpulse : 0;
      lambda = newImpulse - ccp.normalImpulse;
      PX = lambda * normalX;
      PY = lambda * normalY;
      vA.x -= invMassA * PX;
      vA.y -= invMassA * PY;
      wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
      vB.x += invMassB * PX;
      vB.y += invMassB * PY;
      wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
      ccp.normalImpulse = newImpulse
    }else {
      var cp1 = c.points[0];
      var cp2 = c.points[1];
      var aX = cp1.normalImpulse;
      var aY = cp2.normalImpulse;
      var dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
      var dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
      var dv2X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
      var dv2Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
      var vn1 = dv1X * normalX + dv1Y * normalY;
      var vn2 = dv2X * normalX + dv2Y * normalY;
      var bX = vn1 - cp1.velocityBias;
      var bY = vn2 - cp2.velocityBias;
      tMat = c.K;
      bX -= tMat.col1.x * aX + tMat.col2.x * aY;
      bY -= tMat.col1.y * aX + tMat.col2.y * aY;
      var k_errorTol = 0.0010;
      for(;;) {
        tMat = c.normalMass;
        var xX = -(tMat.col1.x * bX + tMat.col2.x * bY);
        var xY = -(tMat.col1.y * bX + tMat.col2.y * bY);
        if(xX >= 0 && xY >= 0) {
          dX = xX - aX;
          dY = xY - aY;
          P1X = dX * normalX;
          P1Y = dX * normalY;
          P2X = dY * normalX;
          P2Y = dY * normalY;
          vA.x -= invMassA * (P1X + P2X);
          vA.y -= invMassA * (P1Y + P2Y);
          wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
          vB.x += invMassB * (P1X + P2X);
          vB.y += invMassB * (P1Y + P2Y);
          wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
          cp1.normalImpulse = xX;
          cp2.normalImpulse = xY;
          break
        }
        xX = -cp1.normalMass * bX;
        xY = 0;
        vn1 = 0;
        vn2 = c.K.col1.y * xX + bY;
        if(xX >= 0 && vn2 >= 0) {
          dX = xX - aX;
          dY = xY - aY;
          P1X = dX * normalX;
          P1Y = dX * normalY;
          P2X = dY * normalX;
          P2Y = dY * normalY;
          vA.x -= invMassA * (P1X + P2X);
          vA.y -= invMassA * (P1Y + P2Y);
          wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
          vB.x += invMassB * (P1X + P2X);
          vB.y += invMassB * (P1Y + P2Y);
          wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
          cp1.normalImpulse = xX;
          cp2.normalImpulse = xY;
          break
        }
        xX = 0;
        xY = -cp2.normalMass * bY;
        vn1 = c.K.col2.x * xY + bX;
        vn2 = 0;
        if(xY >= 0 && vn1 >= 0) {
          dX = xX - aX;
          dY = xY - aY;
          P1X = dX * normalX;
          P1Y = dX * normalY;
          P2X = dY * normalX;
          P2Y = dY * normalY;
          vA.x -= invMassA * (P1X + P2X);
          vA.y -= invMassA * (P1Y + P2Y);
          wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
          vB.x += invMassB * (P1X + P2X);
          vB.y += invMassB * (P1Y + P2Y);
          wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
          cp1.normalImpulse = xX;
          cp2.normalImpulse = xY;
          break
        }
        xX = 0;
        xY = 0;
        vn1 = bX;
        vn2 = bY;
        if(vn1 >= 0 && vn2 >= 0) {
          dX = xX - aX;
          dY = xY - aY;
          P1X = dX * normalX;
          P1Y = dX * normalY;
          P2X = dY * normalX;
          P2Y = dY * normalY;
          vA.x -= invMassA * (P1X + P2X);
          vA.y -= invMassA * (P1Y + P2Y);
          wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
          vB.x += invMassB * (P1X + P2X);
          vB.y += invMassB * (P1Y + P2Y);
          wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
          cp1.normalImpulse = xX;
          cp2.normalImpulse = xY;
          break
        }
        break
      }
    }
    bodyA.m_angularVelocity = wA;
    bodyB.m_angularVelocity = wB
  }
};
b2ContactSolver.prototype.FinalizeVelocityConstraints = function() {
  for(var i = 0;i < this.m_constraintCount;++i) {
    var c = m_constraints[i];
    var m = c.manifold;
    for(var j = 0;j < c.pointCount;++j) {
      var point1 = m.m_points[j];
      var point2 = c.points[j];
      point1.m_normalImpulse = point2.normalImpulse;
      point1.m_tangentImpulse = point2.tangentImpulse
    }
  }
};
b2ContactSolver.prototype.SolvePositionConstraints = function(baumgarte) {
  var minSeparation = 0;
  for(var i = 0;i < this.m_constraintCount;i++) {
    var c = m_constraints[i];
    var bodyA = c.bodyA;
    var bodyB = c.bodyB;
    var invMassA = bodyA.m_mass * bodyA.m_invMass;
    var invIA = bodyA.m_mass * bodyA.m_invI;
    var invMassB = bodyB.m_mass * bodyB.m_invMass;
    var invIB = bodyB.m_mass * bodyB.m_invI;
    b2ContactSolver.s_psm.Initialize(c);
    var normal = b2ContactSolver.s_psm.m_normal;
    for(var j = 0;j < c.pointCount;j++) {
      var ccp = c.points[j];
      var point = b2ContactSolver.s_psm.m_points[j];
      var separation = b2ContactSolver.s_psm.m_separations[j];
      var rAX = point.x - bodyA.m_sweep.c.x;
      var rAY = point.y - bodyA.m_sweep.c.y;
      var rBX = point.x - bodyB.m_sweep.c.x;
      var rBY = point.y - bodyB.m_sweep.c.y;
      minSeparation = minSeparation < separation ? minSeparation : separation;
      var C = b2Math.Clamp(baumgarte * (separation + b2Settings.b2_linearSlop), -b2Settings.b2_maxLinearCorrection, 0);
      var impulse = -ccp.equalizedMass * C;
      var PX = impulse * normal.x;
      var PY = impulse * normal.y;
      bodyA.m_sweep.c.x -= invMassA * PX;
      bodyA.m_sweep.c.y -= invMassA * PY;
      bodyA.m_sweep.a -= invIA * (rAX * PY - rAY * PX);
      bodyA.SynchronizeTransform();
      bodyB.m_sweep.c.x += invMassB * PX;
      bodyB.m_sweep.c.y += invMassB * PY;
      bodyB.m_sweep.a += invIB * (rBX * PY - rBY * PX);
      bodyB.SynchronizeTransform()
    }
  }
  return minSeparation > -1.5 * b2Settings.b2_linearSlop
};var b2Simplex = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Simplex.prototype.__constructor = function() {
  this.m_vertices[0] = this.m_v1;
  this.m_vertices[1] = this.m_v2;
  this.m_vertices[2] = this.m_v3
};
b2Simplex.prototype.__varz = function() {
  this.m_v1 = new b2SimplexVertex;
  this.m_v2 = new b2SimplexVertex;
  this.m_v3 = new b2SimplexVertex;
  this.m_vertices = new Array(3)
};
b2Simplex.prototype.m_v1 = new b2SimplexVertex;
b2Simplex.prototype.m_v2 = new b2SimplexVertex;
b2Simplex.prototype.m_v3 = new b2SimplexVertex;
b2Simplex.prototype.m_vertices = new Array(3);
b2Simplex.prototype.m_count = 0;
b2Simplex.prototype.ReadCache = function(cache, proxyA, transformA, proxyB, transformB) {
  b2Settings.b2Assert(0 <= cache.count && cache.count <= 3);
  var wALocal;
  var wBLocal;
  this.m_count = cache.count;
  var vertices = this.m_vertices;
  for(var i = 0;i < this.m_count;i++) {
    var v = vertices[i];
    v.indexA = cache.indexA[i];
    v.indexB = cache.indexB[i];
    wALocal = proxyA.GetVertex(v.indexA);
    wBLocal = proxyB.GetVertex(v.indexB);
    v.wA = b2Math.MulX(transformA, wALocal);
    v.wB = b2Math.MulX(transformB, wBLocal);
    v.w = b2Math.SubtractVV(v.wB, v.wA);
    v.a = 0
  }
  if(this.m_count > 1) {
    var metric1 = cache.metric;
    var metric2 = this.GetMetric();
    if(metric2 < 0.5 * metric1 || 2 * metric1 < metric2 || metric2 < Number.MIN_VALUE) {
      this.m_count = 0
    }
  }
  if(this.m_count == 0) {
    v = vertices[0];
    v.indexA = 0;
    v.indexB = 0;
    wALocal = proxyA.GetVertex(0);
    wBLocal = proxyB.GetVertex(0);
    v.wA = b2Math.MulX(transformA, wALocal);
    v.wB = b2Math.MulX(transformB, wBLocal);
    v.w = b2Math.SubtractVV(v.wB, v.wA);
    this.m_count = 1
  }
};
b2Simplex.prototype.WriteCache = function(cache) {
  cache.metric = this.GetMetric();
  cache.count = parseInt(this.m_count);
  var vertices = this.m_vertices;
  for(var i = 0;i < this.m_count;i++) {
    cache.indexA[i] = parseInt(vertices[i].indexA);
    cache.indexB[i] = parseInt(vertices[i].indexB)
  }
};
b2Simplex.prototype.GetSearchDirection = function() {
  switch(this.m_count) {
    case 1:
      return this.m_v1.w.GetNegative();
    case 2:
      var e12 = b2Math.SubtractVV(this.m_v2.w, this.m_v1.w);
      var sgn = b2Math.CrossVV(e12, this.m_v1.w.GetNegative());
      if(sgn > 0) {
        return b2Math.CrossFV(1, e12)
      }else {
        return b2Math.CrossVF(e12, 1)
      }
    ;
    default:
      b2Settings.b2Assert(false);
      return new b2Vec2
  }
};
b2Simplex.prototype.GetClosestPoint = function() {
  switch(this.m_count) {
    case 0:
      b2Settings.b2Assert(false);
      return new b2Vec2;
    case 1:
      return this.m_v1.w;
    case 2:
      return new b2Vec2(this.m_v1.a * this.m_v1.w.x + this.m_v2.a * this.m_v2.w.x, this.m_v1.a * this.m_v1.w.y + this.m_v2.a * this.m_v2.w.y);
    default:
      b2Settings.b2Assert(false);
      return new b2Vec2
  }
};
b2Simplex.prototype.GetWitnessPoints = function(pA, pB) {
  switch(this.m_count) {
    case 0:
      b2Settings.b2Assert(false);
      break;
    case 1:
      pA.SetV(this.m_v1.wA);
      pB.SetV(this.m_v1.wB);
      break;
    case 2:
      pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x;
      pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y;
      pB.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x;
      pB.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y;
      break;
    case 3:
      pB.x = pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x + this.m_v3.a * this.m_v3.wA.x;
      pB.y = pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y + this.m_v3.a * this.m_v3.wA.y;
      break;
    default:
      b2Settings.b2Assert(false);
      break
  }
};
b2Simplex.prototype.GetMetric = function() {
  switch(this.m_count) {
    case 0:
      b2Settings.b2Assert(false);
      return 0;
    case 1:
      return 0;
    case 2:
      return b2Math.SubtractVV(this.m_v1.w, this.m_v2.w).Length();
    case 3:
      return b2Math.CrossVV(b2Math.SubtractVV(this.m_v2.w, this.m_v1.w), b2Math.SubtractVV(this.m_v3.w, this.m_v1.w));
    default:
      b2Settings.b2Assert(false);
      return 0
  }
};
b2Simplex.prototype.Solve2 = function() {
  var w1 = this.m_v1.w;
  var w2 = this.m_v2.w;
  var e12 = b2Math.SubtractVV(w2, w1);
  var d12_2 = -(w1.x * e12.x + w1.y * e12.y);
  if(d12_2 <= 0) {
    this.m_v1.a = 1;
    this.m_count = 1;
    return
  }
  var d12_1 = w2.x * e12.x + w2.y * e12.y;
  if(d12_1 <= 0) {
    this.m_v2.a = 1;
    this.m_count = 1;
    this.m_v1.Set(this.m_v2);
    return
  }
  var inv_d12 = 1 / (d12_1 + d12_2);
  this.m_v1.a = d12_1 * inv_d12;
  this.m_v2.a = d12_2 * inv_d12;
  this.m_count = 2
};
b2Simplex.prototype.Solve3 = function() {
  var w1 = this.m_v1.w;
  var w2 = this.m_v2.w;
  var w3 = this.m_v3.w;
  var e12 = b2Math.SubtractVV(w2, w1);
  var w1e12 = b2Math.Dot(w1, e12);
  var w2e12 = b2Math.Dot(w2, e12);
  var d12_1 = w2e12;
  var d12_2 = -w1e12;
  var e13 = b2Math.SubtractVV(w3, w1);
  var w1e13 = b2Math.Dot(w1, e13);
  var w3e13 = b2Math.Dot(w3, e13);
  var d13_1 = w3e13;
  var d13_2 = -w1e13;
  var e23 = b2Math.SubtractVV(w3, w2);
  var w2e23 = b2Math.Dot(w2, e23);
  var w3e23 = b2Math.Dot(w3, e23);
  var d23_1 = w3e23;
  var d23_2 = -w2e23;
  var n123 = b2Math.CrossVV(e12, e13);
  var d123_1 = n123 * b2Math.CrossVV(w2, w3);
  var d123_2 = n123 * b2Math.CrossVV(w3, w1);
  var d123_3 = n123 * b2Math.CrossVV(w1, w2);
  if(d12_2 <= 0 && d13_2 <= 0) {
    this.m_v1.a = 1;
    this.m_count = 1;
    return
  }
  if(d12_1 > 0 && d12_2 > 0 && d123_3 <= 0) {
    var inv_d12 = 1 / (d12_1 + d12_2);
    this.m_v1.a = d12_1 * inv_d12;
    this.m_v2.a = d12_2 * inv_d12;
    this.m_count = 2;
    return
  }
  if(d13_1 > 0 && d13_2 > 0 && d123_2 <= 0) {
    var inv_d13 = 1 / (d13_1 + d13_2);
    this.m_v1.a = d13_1 * inv_d13;
    this.m_v3.a = d13_2 * inv_d13;
    this.m_count = 2;
    this.m_v2.Set(this.m_v3);
    return
  }
  if(d12_1 <= 0 && d23_2 <= 0) {
    this.m_v2.a = 1;
    this.m_count = 1;
    this.m_v1.Set(this.m_v2);
    return
  }
  if(d13_1 <= 0 && d23_1 <= 0) {
    this.m_v3.a = 1;
    this.m_count = 1;
    this.m_v1.Set(this.m_v3);
    return
  }
  if(d23_1 > 0 && d23_2 > 0 && d123_1 <= 0) {
    var inv_d23 = 1 / (d23_1 + d23_2);
    this.m_v2.a = d23_1 * inv_d23;
    this.m_v3.a = d23_2 * inv_d23;
    this.m_count = 2;
    this.m_v1.Set(this.m_v3);
    return
  }
  var inv_d123 = 1 / (d123_1 + d123_2 + d123_3);
  this.m_v1.a = d123_1 * inv_d123;
  this.m_v2.a = d123_2 * inv_d123;
  this.m_v3.a = d123_3 * inv_d123;
  this.m_count = 3
};var b2RevoluteJointDef = function() {
  b2JointDef.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2RevoluteJointDef.prototype, b2JointDef.prototype);
b2RevoluteJointDef.prototype._super = function() {
  b2JointDef.prototype.__constructor.apply(this, arguments)
};
b2RevoluteJointDef.prototype.__constructor = function() {
  this.type = b2Joint.e_revoluteJoint;
  this.localAnchorA.Set(0, 0);
  this.localAnchorB.Set(0, 0);
  this.referenceAngle = 0;
  this.lowerAngle = 0;
  this.upperAngle = 0;
  this.maxMotorTorque = 0;
  this.motorSpeed = 0;
  this.enableLimit = false;
  this.enableMotor = false
};
b2RevoluteJointDef.prototype.__varz = function() {
  this.localAnchorA = new b2Vec2;
  this.localAnchorB = new b2Vec2
};
b2RevoluteJointDef.prototype.localAnchorA = new b2Vec2;
b2RevoluteJointDef.prototype.localAnchorB = new b2Vec2;
b2RevoluteJointDef.prototype.referenceAngle = null;
b2RevoluteJointDef.prototype.enableLimit = null;
b2RevoluteJointDef.prototype.lowerAngle = null;
b2RevoluteJointDef.prototype.upperAngle = null;
b2RevoluteJointDef.prototype.enableMotor = null;
b2RevoluteJointDef.prototype.motorSpeed = null;
b2RevoluteJointDef.prototype.maxMotorTorque = null;
b2RevoluteJointDef.prototype.Initialize = function(bA, bB, anchor) {
  this.bodyA = bA;
  this.bodyB = bB;
  this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
  this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
  this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle()
};var b2WeldJoint = function() {
  b2Joint.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2WeldJoint.prototype, b2Joint.prototype);
b2WeldJoint.prototype._super = function() {
  b2Joint.prototype.__constructor.apply(this, arguments)
};
b2WeldJoint.prototype.__constructor = function(def) {
  this._super(def);
  this.m_localAnchorA.SetV(def.localAnchorA);
  this.m_localAnchorB.SetV(def.localAnchorB);
  this.m_referenceAngle = def.referenceAngle;
  this.m_impulse.SetZero();
  this.m_mass = new b2Mat33
};
b2WeldJoint.prototype.__varz = function() {
  this.m_localAnchorA = new b2Vec2;
  this.m_localAnchorB = new b2Vec2;
  this.m_impulse = new b2Vec3;
  this.m_mass = new b2Mat33
};
b2WeldJoint.prototype.m_localAnchorA = new b2Vec2;
b2WeldJoint.prototype.m_localAnchorB = new b2Vec2;
b2WeldJoint.prototype.m_referenceAngle = null;
b2WeldJoint.prototype.m_impulse = new b2Vec3;
b2WeldJoint.prototype.m_mass = new b2Mat33;
b2WeldJoint.prototype.GetAnchorA = function() {
  return m_bodyA.GetWorldPoint(this.m_localAnchorA)
};
b2WeldJoint.prototype.GetAnchorB = function() {
  return m_bodyB.GetWorldPoint(this.m_localAnchorB)
};
b2WeldJoint.prototype.GetReactionForce = function(inv_dt) {
  return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y)
};
b2WeldJoint.prototype.GetReactionTorque = function(inv_dt) {
  return inv_dt * this.m_impulse.z
};var b2MouseJointDef = function() {
  b2JointDef.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2MouseJointDef.prototype, b2JointDef.prototype);
b2MouseJointDef.prototype._super = function() {
  b2JointDef.prototype.__constructor.apply(this, arguments)
};
b2MouseJointDef.prototype.__constructor = function() {
  this.type = b2Joint.e_mouseJoint;
  this.maxForce = 0;
  this.frequencyHz = 5;
  this.dampingRatio = 0.7
};
b2MouseJointDef.prototype.__varz = function() {
  this.target = new b2Vec2
};
b2MouseJointDef.prototype.target = new b2Vec2;
b2MouseJointDef.prototype.maxForce = null;
b2MouseJointDef.prototype.frequencyHz = null;
b2MouseJointDef.prototype.dampingRatio = null;var b2Math = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Math.prototype.__constructor = function() {
};
b2Math.prototype.__varz = function() {
};
b2Math.b2Vec2_zero = new b2Vec2(0, 0);
b2Math.b2Mat22_identity = b2Mat22.FromVV(new b2Vec2(1, 0), new b2Vec2(0, 1));
b2Math.b2Transform_identity = new b2Transform(b2Math.b2Vec2_zero, b2Math.b2Mat22_identity);
b2Math.IsValid = function(x) {
  return isFinite(x)
};
b2Math.Dot = function(a, b) {
  return a.x * b.x + a.y * b.y
};
b2Math.CrossVV = function(a, b) {
  return a.x * b.y - a.y * b.x
};
b2Math.CrossVF = function(a, s) {
  var v = new b2Vec2(s * a.y, -s * a.x);
  return v
};
b2Math.CrossFV = function(s, a) {
  var v = new b2Vec2(-s * a.y, s * a.x);
  return v
};
b2Math.MulMV = function(A, v) {
  var u = new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
  return u
};
b2Math.MulTMV = function(A, v) {
  var u = new b2Vec2(b2Math.Dot(v, A.col1), b2Math.Dot(v, A.col2));
  return u
};
b2Math.MulX = function(T, v) {
  var a = b2Math.MulMV(T.R, v);
  a.x += T.position.x;
  a.y += T.position.y;
  return a
};
b2Math.MulXT = function(T, v) {
  var a = b2Math.SubtractVV(v, T.position);
  var tX = a.x * T.R.col1.x + a.y * T.R.col1.y;
  a.y = a.x * T.R.col2.x + a.y * T.R.col2.y;
  a.x = tX;
  return a
};
b2Math.AddVV = function(a, b) {
  var v = new b2Vec2(a.x + b.x, a.y + b.y);
  return v
};
b2Math.SubtractVV = function(a, b) {
  var v = new b2Vec2(a.x - b.x, a.y - b.y);
  return v
};
b2Math.Distance = function(a, b) {
  var cX = a.x - b.x;
  var cY = a.y - b.y;
  return Math.sqrt(cX * cX + cY * cY)
};
b2Math.DistanceSquared = function(a, b) {
  var cX = a.x - b.x;
  var cY = a.y - b.y;
  return cX * cX + cY * cY
};
b2Math.MulFV = function(s, a) {
  var v = new b2Vec2(s * a.x, s * a.y);
  return v
};
b2Math.AddMM = function(A, B) {
  var C = b2Mat22.FromVV(b2Math.AddVV(A.col1, B.col1), b2Math.AddVV(A.col2, B.col2));
  return C
};
b2Math.MulMM = function(A, B) {
  var C = b2Mat22.FromVV(b2Math.MulMV(A, B.col1), b2Math.MulMV(A, B.col2));
  return C
};
b2Math.MulTMM = function(A, B) {
  var c1 = new b2Vec2(b2Math.Dot(A.col1, B.col1), b2Math.Dot(A.col2, B.col1));
  var c2 = new b2Vec2(b2Math.Dot(A.col1, B.col2), b2Math.Dot(A.col2, B.col2));
  var C = b2Mat22.FromVV(c1, c2);
  return C
};
b2Math.Abs = function(a) {
  return a > 0 ? a : -a
};
b2Math.AbsV = function(a) {
  var b = new b2Vec2(b2Math.Abs(a.x), b2Math.Abs(a.y));
  return b
};
b2Math.AbsM = function(A) {
  var B = b2Mat22.FromVV(b2Math.AbsV(A.col1), b2Math.AbsV(A.col2));
  return B
};
b2Math.Min = function(a, b) {
  return a < b ? a : b
};
b2Math.MinV = function(a, b) {
  var c = new b2Vec2(b2Math.Min(a.x, b.x), b2Math.Min(a.y, b.y));
  return c
};
b2Math.Max = function(a, b) {
  return a > b ? a : b
};
b2Math.MaxV = function(a, b) {
  var c = new b2Vec2(b2Math.Max(a.x, b.x), b2Math.Max(a.y, b.y));
  return c
};
b2Math.Clamp = function(a, low, high) {
  return a < low ? low : a > high ? high : a
};
b2Math.ClampV = function(a, low, high) {
  return b2Math.MaxV(low, b2Math.MinV(a, high))
};
b2Math.Swap = function(a, b) {
  var tmp = a[0];
  a[0] = b[0];
  b[0] = tmp
};
b2Math.Random = function() {
  return Math.random() * 2 - 1
};
b2Math.RandomRange = function(lo, hi) {
  var r = Math.random();
  r = (hi - lo) * r + lo;
  return r
};
b2Math.NextPowerOfTwo = function(x) {
  x |= x >> 1 & 2147483647;
  x |= x >> 2 & 1073741823;
  x |= x >> 4 & 268435455;
  x |= x >> 8 & 16777215;
  x |= x >> 16 & 65535;
  return x + 1
};
b2Math.IsPowerOfTwo = function(x) {
  var result = x > 0 && (x & x - 1) == 0;
  return result
};var b2PulleyJoint = function() {
  b2Joint.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2PulleyJoint.prototype, b2Joint.prototype);
b2PulleyJoint.prototype._super = function() {
  b2Joint.prototype.__constructor.apply(this, arguments)
};
b2PulleyJoint.prototype.__constructor = function(def) {
  this._super(def);
  var tMat;
  var tX;
  var tY;
  this.m_ground = m_bodyA.m_world.m_groundBody;
  this.m_groundAnchor1.x = def.groundAnchorA.x - this.m_ground.m_xf.position.x;
  this.m_groundAnchor1.y = def.groundAnchorA.y - this.m_ground.m_xf.position.y;
  this.m_groundAnchor2.x = def.groundAnchorB.x - this.m_ground.m_xf.position.x;
  this.m_groundAnchor2.y = def.groundAnchorB.y - this.m_ground.m_xf.position.y;
  this.m_localAnchor1.SetV(def.localAnchorA);
  this.m_localAnchor2.SetV(def.localAnchorB);
  this.m_ratio = def.ratio;
  this.m_constant = def.lengthA + this.m_ratio * def.lengthB;
  this.m_maxLength1 = b2Math.Min(def.maxLengthA, this.m_constant - this.m_ratio * b2_minPulleyLength);
  this.m_maxLength2 = b2Math.Min(def.maxLengthB, (this.m_constant - b2_minPulleyLength) / this.m_ratio);
  this.m_impulse = 0;
  this.m_limitImpulse1 = 0;
  this.m_limitImpulse2 = 0
};
b2PulleyJoint.prototype.__varz = function() {
  this.m_groundAnchor1 = new b2Vec2;
  this.m_groundAnchor2 = new b2Vec2;
  this.m_localAnchor1 = new b2Vec2;
  this.m_localAnchor2 = new b2Vec2;
  this.m_u1 = new b2Vec2;
  this.m_u2 = new b2Vec2
};
b2PulleyJoint.prototype.m_ground = null;
b2PulleyJoint.prototype.m_groundAnchor1 = new b2Vec2;
b2PulleyJoint.prototype.m_groundAnchor2 = new b2Vec2;
b2PulleyJoint.prototype.m_localAnchor1 = new b2Vec2;
b2PulleyJoint.prototype.m_localAnchor2 = new b2Vec2;
b2PulleyJoint.prototype.m_u1 = new b2Vec2;
b2PulleyJoint.prototype.m_u2 = new b2Vec2;
b2PulleyJoint.prototype.m_constant = null;
b2PulleyJoint.prototype.m_ratio = null;
b2PulleyJoint.prototype.m_maxLength1 = null;
b2PulleyJoint.prototype.m_maxLength2 = null;
b2PulleyJoint.prototype.m_pulleyMass = null;
b2PulleyJoint.prototype.m_limitMass1 = null;
b2PulleyJoint.prototype.m_limitMass2 = null;
b2PulleyJoint.prototype.m_impulse = null;
b2PulleyJoint.prototype.m_limitImpulse1 = null;
b2PulleyJoint.prototype.m_limitImpulse2 = null;
b2PulleyJoint.prototype.m_state = 0;
b2PulleyJoint.prototype.m_limitState1 = 0;
b2PulleyJoint.prototype.m_limitState2 = 0;
b2PulleyJoint.prototype.GetAnchorA = function() {
  return m_bodyA.GetWorldPoint(this.m_localAnchor1)
};
b2PulleyJoint.prototype.GetAnchorB = function() {
  return m_bodyB.GetWorldPoint(this.m_localAnchor2)
};
b2PulleyJoint.prototype.GetReactionForce = function(inv_dt) {
  return new b2Vec2(inv_dt * this.m_impulse * this.m_u2.x, inv_dt * this.m_impulse * this.m_u2.y)
};
b2PulleyJoint.prototype.GetReactionTorque = function(inv_dt) {
  return 0
};
b2PulleyJoint.prototype.GetGroundAnchorA = function() {
  var a = this.m_ground.m_xf.position.Copy();
  a.Add(this.m_groundAnchor1);
  return a
};
b2PulleyJoint.prototype.GetGroundAnchorB = function() {
  var a = this.m_ground.m_xf.position.Copy();
  a.Add(this.m_groundAnchor2);
  return a
};
b2PulleyJoint.prototype.GetLength1 = function() {
  var p = m_bodyA.GetWorldPoint(this.m_localAnchor1);
  var sX = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
  var sY = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
  var dX = p.x - sX;
  var dY = p.y - sY;
  return Math.sqrt(dX * dX + dY * dY)
};
b2PulleyJoint.prototype.GetLength2 = function() {
  var p = m_bodyB.GetWorldPoint(this.m_localAnchor2);
  var sX = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
  var sY = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
  var dX = p.x - sX;
  var dY = p.y - sY;
  return Math.sqrt(dX * dX + dY * dY)
};
b2PulleyJoint.prototype.GetRatio = function() {
  return this.m_ratio
};var b2Color = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Color.prototype.__constructor = function(rr, gg, bb) {
  this._r = parseInt(255 * b2Math.Clamp(rr, 0, 1));
  this._g = parseInt(255 * b2Math.Clamp(gg, 0, 1));
  this._b = parseInt(255 * b2Math.Clamp(bb, 0, 1))
};
b2Color.prototype.__varz = function() {
};
b2Color.prototype._r = 0;
b2Color.prototype._g = 0;
b2Color.prototype._b = 0;
b2Color.prototype.Set = function(rr, gg, bb) {
  this._r = parseInt(255 * b2Math.Clamp(rr, 0, 1));
  this._g = parseInt(255 * b2Math.Clamp(gg, 0, 1));
  this._b = parseInt(255 * b2Math.Clamp(bb, 0, 1))
};
b2Color.prototype.__defineSetter__("r", function(rr) {
  this._r = parseInt(255 * b2Math.Clamp(rr, 0, 1))
});
b2Color.prototype.__defineSetter__("g", function(gg) {
  this._g = parseInt(255 * b2Math.Clamp(gg, 0, 1))
});
b2Color.prototype.__defineSetter__("b", function(bb) {
  this._b = parseInt(255 * b2Math.Clamp(bb, 0, 1))
});
b2Color.prototype.__defineGetter__("color", function() {
  return this._r << 16 | this._g << 8 | this._b
});var b2Distance = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Distance.prototype.__constructor = function() {
};
b2Distance.prototype.__varz = function() {
};
b2Distance.b2_gjkCalls = 0;
b2Distance.b2_gjkIters = 0;
b2Distance.b2_gjkMaxIters = 0;
b2Distance.s_simplex = new b2Simplex;
b2Distance.s_saveA = new Array(3);
b2Distance.s_saveB = new Array(3);
b2Distance.Distance = function(output, cache, input) {
  ++b2Distance.b2_gjkCalls;
  var proxyA = input.proxyA;
  var proxyB = input.proxyB;
  var transformA = input.transformA;
  var transformB = input.transformB;
  var simplex = b2Distance.s_simplex;
  simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);
  var vertices = simplex.m_vertices;
  var k_maxIters = 20;
  var saveA = b2Distance.s_saveA;
  var saveB = b2Distance.s_saveB;
  var saveCount = 0;
  var closestPoint = simplex.GetClosestPoint();
  var distanceSqr1 = closestPoint.LengthSquared();
  var distanceSqr2 = distanceSqr1;
  var i = 0;
  var p;
  var iter = 0;
  while(iter < k_maxIters) {
    saveCount = simplex.m_count;
    for(i = 0;i < saveCount;i++) {
      saveA[i] = vertices[i].indexA;
      saveB[i] = vertices[i].indexB
    }
    switch(simplex.m_count) {
      case 1:
        break;
      case 2:
        simplex.Solve2();
        break;
      case 3:
        simplex.Solve3();
        break;
      default:
        b2Settings.b2Assert(false)
    }
    if(simplex.m_count == 3) {
      break
    }
    p = simplex.GetClosestPoint();
    distanceSqr2 = p.LengthSquared();
    if(distanceSqr2 > distanceSqr1) {
    }
    distanceSqr1 = distanceSqr2;
    var d = simplex.GetSearchDirection();
    if(d.LengthSquared() < Number.MIN_VALUE * Number.MIN_VALUE) {
      break
    }
    var vertex = vertices[simplex.m_count];
    vertex.indexA = proxyA.GetSupport(b2Math.MulTMV(transformA.R, d.GetNegative()));
    vertex.wA = b2Math.MulX(transformA, proxyA.GetVertex(vertex.indexA));
    vertex.indexB = proxyB.GetSupport(b2Math.MulTMV(transformB.R, d));
    vertex.wB = b2Math.MulX(transformB, proxyB.GetVertex(vertex.indexB));
    vertex.w = b2Math.SubtractVV(vertex.wB, vertex.wA);
    ++iter;
    ++b2Distance.b2_gjkIters;
    var duplicate = false;
    for(i = 0;i < saveCount;i++) {
      if(vertex.indexA == saveA[i] && vertex.indexB == saveB[i]) {
        duplicate = true;
        break
      }
    }
    if(duplicate) {
      break
    }
    ++simplex.m_count
  }
  b2Distance.b2_gjkMaxIters = b2Math.Max(b2Distance.b2_gjkMaxIters, iter);
  simplex.GetWitnessPoints(output.pointA, output.pointB);
  output.distance = b2Math.SubtractVV(output.pointA, output.pointB).Length();
  output.iterations = iter;
  simplex.WriteCache(cache);
  if(input.useRadii) {
    var rA = proxyA.m_radius;
    var rB = proxyB.m_radius;
    if(output.distance > rA + rB && output.distance > Number.MIN_VALUE) {
      output.distance -= rA + rB;
      var normal = b2Math.SubtractVV(output.pointB, output.pointA);
      normal.Normalize();
      output.pointA.x += rA * normal.x;
      output.pointA.y += rA * normal.y;
      output.pointB.x -= rB * normal.x;
      output.pointB.y -= rB * normal.y
    }else {
      p = new b2Vec2;
      p.x = 0.5 * (output.pointA.x + output.pointB.x);
      p.y = 0.5 * (output.pointA.y + output.pointB.y);
      output.pointA.x = output.pointB.x = p.x;
      output.pointA.y = output.pointB.y = p.y;
      output.distance = 0
    }
  }
};var b2TimeOfImpact = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2TimeOfImpact.prototype.__constructor = function() {
};
b2TimeOfImpact.prototype.__varz = function() {
};
b2TimeOfImpact.b2_toiCalls = 0;
b2TimeOfImpact.b2_toiIters = 0;
b2TimeOfImpact.b2_toiMaxIters = 0;
b2TimeOfImpact.b2_toiRootIters = 0;
b2TimeOfImpact.b2_toiMaxRootIters = 0;
b2TimeOfImpact.s_cache = new b2SimplexCache;
b2TimeOfImpact.s_distanceInput = new b2DistanceInput;
b2TimeOfImpact.s_xfA = new b2Transform;
b2TimeOfImpact.s_xfB = new b2Transform;
b2TimeOfImpact.s_fcn = new b2SeparationFunction;
b2TimeOfImpact.s_distanceOutput = new b2DistanceOutput;
b2TimeOfImpact.TimeOfImpact = function(input) {
  ++b2TimeOfImpact.b2_toiCalls;
  var proxyA = input.proxyA;
  var proxyB = input.proxyB;
  var sweepA = input.sweepA;
  var sweepB = input.sweepB;
  b2Settings.b2Assert(sweepA.t0 == sweepB.t0);
  b2Settings.b2Assert(1 - sweepA.t0 > Number.MIN_VALUE);
  var radius = proxyA.m_radius + proxyB.m_radius;
  var tolerance = input.tolerance;
  var alpha = 0;
  var k_maxIterations = 1E3;
  var iter = 0;
  var target = 0;
  b2TimeOfImpact.s_cache.count = 0;
  b2TimeOfImpact.s_distanceInput.useRadii = false;
  for(;;) {
    sweepA.GetTransform(b2TimeOfImpact.s_xfA, alpha);
    sweepB.GetTransform(b2TimeOfImpact.s_xfB, alpha);
    b2TimeOfImpact.s_distanceInput.proxyA = proxyA;
    b2TimeOfImpact.s_distanceInput.proxyB = proxyB;
    b2TimeOfImpact.s_distanceInput.transformA = b2TimeOfImpact.s_xfA;
    b2TimeOfImpact.s_distanceInput.transformB = b2TimeOfImpact.s_xfB;
    b2Distance.Distance(b2TimeOfImpact.s_distanceOutput, b2TimeOfImpact.s_cache, b2TimeOfImpact.s_distanceInput);
    if(b2TimeOfImpact.s_distanceOutput.distance <= 0) {
      alpha = 1;
      break
    }
    b2TimeOfImpact.s_fcn.Initialize(b2TimeOfImpact.s_cache, proxyA, b2TimeOfImpact.s_xfA, proxyB, b2TimeOfImpact.s_xfB);
    var separation = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);
    if(separation <= 0) {
      alpha = 1;
      break
    }
    if(iter == 0) {
      if(separation > radius) {
        target = b2Math.Max(radius - tolerance, 0.75 * radius)
      }else {
        target = b2Math.Max(separation - tolerance, 0.02 * radius)
      }
    }
    if(separation - target < 0.5 * tolerance) {
      if(iter == 0) {
        alpha = 1;
        break
      }
      break
    }
    var newAlpha = alpha;
    var x1 = alpha;
    var x2 = 1;
    var f1 = separation;
    sweepA.GetTransform(b2TimeOfImpact.s_xfA, x2);
    sweepB.GetTransform(b2TimeOfImpact.s_xfB, x2);
    var f2 = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);
    if(f2 >= target) {
      alpha = 1;
      break
    }
    var rootIterCount = 0;
    for(;;) {
      var x;
      if(rootIterCount & 1) {
        x = x1 + (target - f1) * (x2 - x1) / (f2 - f1)
      }else {
        x = 0.5 * (x1 + x2)
      }
      sweepA.GetTransform(b2TimeOfImpact.s_xfA, x);
      sweepB.GetTransform(b2TimeOfImpact.s_xfB, x);
      var f = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);
      if(b2Math.Abs(f - target) < 0.025 * tolerance) {
        newAlpha = x;
        break
      }
      if(f > target) {
        x1 = x;
        f1 = f
      }else {
        x2 = x;
        f2 = f
      }
      ++rootIterCount;
      ++b2TimeOfImpact.b2_toiRootIters;
      if(rootIterCount == 50) {
        break
      }
    }
    b2TimeOfImpact.b2_toiMaxRootIters = b2Math.Max(b2TimeOfImpact.b2_toiMaxRootIters, rootIterCount);
    if(newAlpha < (1 + 100 * Number.MIN_VALUE) * alpha) {
      break
    }
    alpha = newAlpha;
    iter++;
    ++b2TimeOfImpact.b2_toiIters;
    if(iter == k_maxIterations) {
      break
    }
  }
  b2TimeOfImpact.b2_toiMaxIters = b2Math.Max(b2TimeOfImpact.b2_toiMaxIters, iter);
  return alpha
};var b2TOIInput = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2TOIInput.prototype.__constructor = function() {
};
b2TOIInput.prototype.__varz = function() {
  this.proxyA = new b2DistanceProxy;
  this.proxyB = new b2DistanceProxy;
  this.sweepA = new b2Sweep;
  this.sweepB = new b2Sweep
};
b2TOIInput.prototype.proxyA = new b2DistanceProxy;
b2TOIInput.prototype.proxyB = new b2DistanceProxy;
b2TOIInput.prototype.sweepA = new b2Sweep;
b2TOIInput.prototype.sweepB = new b2Sweep;
b2TOIInput.prototype.tolerance = null;var b2Contact = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Contact.prototype.__constructor = function() {
};
b2Contact.prototype.__varz = function() {
};
b2Contact.s_input = new b2TOIInput;
b2Contact.prototype.GetManifold = function() {
  return m_manifold
};
b2Contact.prototype.GetWorldManifold = function(worldManifold) {
  var bodyA = m_fixtureA.GetBody();
  var bodyB = m_fixtureB.GetBody();
  var shapeA = m_fixtureA.GetShape();
  var shapeB = m_fixtureB.GetShape();
  worldManifold.Initialize(m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius)
};
b2Contact.prototype.IsTouching = function() {
  return(m_flags & e_touchingFlag) == e_touchingFlag
};
b2Contact.prototype.IsContinuous = function() {
  return(m_flags & e_continuousFlag) == e_continuousFlag
};
b2Contact.prototype.SetSensor = function(sensor) {
  if(sensor) {
    m_flags |= e_sensorFlag
  }else {
    m_flags &= ~e_sensorFlag
  }
};
b2Contact.prototype.IsSensor = function() {
  return(m_flags & e_sensorFlag) == e_sensorFlag
};
b2Contact.prototype.SetEnabled = function(flag) {
  if(flag) {
    m_flags |= e_enabledFlag
  }else {
    m_flags &= ~e_enabledFlag
  }
};
b2Contact.prototype.IsEnabled = function() {
  return(m_flags & e_enabledFlag) == e_enabledFlag
};
b2Contact.prototype.GetNext = function() {
  return m_next
};
b2Contact.prototype.GetFixtureA = function() {
  return m_fixtureA
};
b2Contact.prototype.GetFixtureB = function() {
  return m_fixtureB
};
b2Contact.prototype.FlagForFiltering = function() {
  m_flags |= e_filterFlag
};var b2ContactConstraint = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ContactConstraint.prototype.__constructor = function() {
  this.points = new Array(b2Settings.b2_maxManifoldPoints);
  for(var i = 0;i < b2Settings.b2_maxManifoldPoints;i++) {
    this.points[i] = new b2ContactConstraintPoint
  }
};
b2ContactConstraint.prototype.__varz = function() {
  this.localPlaneNormal = new b2Vec2;
  this.localPoint = new b2Vec2;
  this.normal = new b2Vec2;
  this.normalMass = new b2Mat22;
  this.K = new b2Mat22
};
b2ContactConstraint.prototype.points = null;
b2ContactConstraint.prototype.localPlaneNormal = new b2Vec2;
b2ContactConstraint.prototype.localPoint = new b2Vec2;
b2ContactConstraint.prototype.normal = new b2Vec2;
b2ContactConstraint.prototype.normalMass = new b2Mat22;
b2ContactConstraint.prototype.K = new b2Mat22;
b2ContactConstraint.prototype.bodyA = null;
b2ContactConstraint.prototype.bodyB = null;
b2ContactConstraint.prototype.type = 0;
b2ContactConstraint.prototype.radius = null;
b2ContactConstraint.prototype.friction = null;
b2ContactConstraint.prototype.restitution = null;
b2ContactConstraint.prototype.pointCount = 0;
b2ContactConstraint.prototype.manifold = null;var b2ContactResult = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ContactResult.prototype.__constructor = function() {
};
b2ContactResult.prototype.__varz = function() {
  this.position = new b2Vec2;
  this.normal = new b2Vec2;
  this.id = new b2ContactID
};
b2ContactResult.prototype.shape1 = null;
b2ContactResult.prototype.shape2 = null;
b2ContactResult.prototype.position = new b2Vec2;
b2ContactResult.prototype.normal = new b2Vec2;
b2ContactResult.prototype.normalImpulse = null;
b2ContactResult.prototype.tangentImpulse = null;
b2ContactResult.prototype.id = new b2ContactID;var b2PolygonContact = function() {
  b2Contact.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2PolygonContact.prototype, b2Contact.prototype);
b2PolygonContact.prototype._super = function() {
  b2Contact.prototype.__constructor.apply(this, arguments)
};
b2PolygonContact.prototype.__constructor = function() {
};
b2PolygonContact.prototype.__varz = function() {
};
b2PolygonContact.Create = function(allocator) {
  return new b2PolygonContact
};
b2PolygonContact.Destroy = function(contact, allocator) {
};
b2PolygonContact.prototype.Reset = function(fixtureA, fixtureB) {
  this._super.Reset(fixtureA, fixtureB)
};var ClipVertex = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
ClipVertex.prototype.__constructor = function() {
};
ClipVertex.prototype.__varz = function() {
  this.v = new b2Vec2;
  this.id = new b2ContactID
};
ClipVertex.prototype.v = new b2Vec2;
ClipVertex.prototype.id = new b2ContactID;
ClipVertex.prototype.Set = function(other) {
  this.v.SetV(other.v);
  this.id.Set(other.id)
};var b2ManifoldPoint = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ManifoldPoint.prototype.__constructor = function() {
  this.Reset()
};
b2ManifoldPoint.prototype.__varz = function() {
  this.m_localPoint = new b2Vec2;
  this.m_id = new b2ContactID
};
b2ManifoldPoint.prototype.m_localPoint = new b2Vec2;
b2ManifoldPoint.prototype.m_normalImpulse = null;
b2ManifoldPoint.prototype.m_tangentImpulse = null;
b2ManifoldPoint.prototype.m_id = new b2ContactID;
b2ManifoldPoint.prototype.Reset = function() {
  this.m_localPoint.SetZero();
  this.m_normalImpulse = 0;
  this.m_tangentImpulse = 0;
  this.m_id.key = 0
};
b2ManifoldPoint.prototype.Set = function(m) {
  this.m_localPoint.SetV(m.m_localPoint);
  this.m_normalImpulse = m.m_normalImpulse;
  this.m_tangentImpulse = m.m_tangentImpulse;
  this.m_id.Set(m.m_id)
};var b2Manifold = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Manifold.prototype.__constructor = function() {
  this.m_points = new Array(b2Settings.b2_maxManifoldPoints);
  for(var i = 0;i < b2Settings.b2_maxManifoldPoints;i++) {
    this.m_points[i] = new b2ManifoldPoint
  }
  this.m_localPlaneNormal = new b2Vec2;
  this.m_localPoint = new b2Vec2
};
b2Manifold.prototype.__varz = function() {
};
b2Manifold.e_circles = 1;
b2Manifold.e_faceA = 2;
b2Manifold.e_faceB = 4;
b2Manifold.prototype.m_points = null;
b2Manifold.prototype.m_localPlaneNormal = null;
b2Manifold.prototype.m_localPoint = null;
b2Manifold.prototype.m_type = 0;
b2Manifold.prototype.m_pointCount = 0;
b2Manifold.prototype.Reset = function() {
  for(var i = 0;i < b2Settings.b2_maxManifoldPoints;i++) {
    this.m_points[i].Reset()
  }
  this.m_localPlaneNormal.SetZero();
  this.m_localPoint.SetZero();
  this.m_type = 0;
  this.m_pointCount = 0
};
b2Manifold.prototype.Set = function(m) {
  this.m_pointCount = m.m_pointCount;
  for(var i = 0;i < b2Settings.b2_maxManifoldPoints;i++) {
    this.m_points[i].Set(m.m_points[i])
  }
  this.m_localPlaneNormal.SetV(m.m_localPlaneNormal);
  this.m_localPoint.SetV(m.m_localPoint);
  this.m_type = m.m_type
};
b2Manifold.prototype.Copy = function() {
  var copy = new b2Manifold;
  copy.Set(this);
  return copy
};var b2NullContact = function() {
  b2Contact.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2NullContact.prototype, b2Contact.prototype);
b2NullContact.prototype._super = function() {
  b2Contact.prototype.__constructor.apply(this, arguments)
};
b2NullContact.prototype.__constructor = function() {
};
b2NullContact.prototype.__varz = function() {
};var b2ContactListener = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ContactListener.prototype.__constructor = function() {
};
b2ContactListener.prototype.__varz = function() {
};
b2ContactListener.b2_defaultListener = new b2ContactListener;
b2ContactListener.prototype.BeginContact = function(contact) {
};
b2ContactListener.prototype.EndContact = function(contact) {
};
b2ContactListener.prototype.PreSolve = function(contact, oldManifold) {
};
b2ContactListener.prototype.PostSolve = function(contact, impulse) {
};var b2Island = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2Island.prototype.__constructor = function() {
  m_bodies = new Array;
  m_contacts = new Array;
  m_joints = new Array
};
b2Island.prototype.__varz = function() {
};
b2Island.s_impulse = new b2ContactImpulse;
b2Island.prototype.m_allocator = null;
b2Island.prototype.m_listener = null;
b2Island.prototype.m_contactSolver = null;
b2Island.prototype.m_bodyCapacity = 0;
b2Island.prototype.Initialize = function(bodyCapacity, contactCapacity, jointCapacity, allocator, listener, contactSolver) {
  var i = 0;
  this.m_bodyCapacity = bodyCapacity;
  m_contactCapacity = contactCapacity;
  m_jointCapacity = jointCapacity;
  m_bodyCount = 0;
  m_contactCount = 0;
  m_jointCount = 0;
  this.m_allocator = allocator;
  this.m_listener = listener;
  this.m_contactSolver = contactSolver;
  for(i = m_bodies.length;i < bodyCapacity;i++) {
    m_bodies[i] = null
  }
  for(i = m_contacts.length;i < contactCapacity;i++) {
    m_contacts[i] = null
  }
  for(i = m_joints.length;i < jointCapacity;i++) {
    m_joints[i] = null
  }
};
b2Island.prototype.Clear = function() {
  m_bodyCount = 0;
  m_contactCount = 0;
  m_jointCount = 0
};
b2Island.prototype.Solve = function(step, gravity, allowSleep) {
  var i = 0;
  var j = 0;
  var b;
  var joint;
  for(i = 0;i < m_bodyCount;++i) {
    b = m_bodies[i];
    if(b.GetType() != b2Body.b2_dynamicBody) {
      continue
    }
    b.m_linearVelocity.x += step.dt * (gravity.x + b.m_invMass * b.m_force.x);
    b.m_linearVelocity.y += step.dt * (gravity.y + b.m_invMass * b.m_force.y);
    b.m_angularVelocity += step.dt * b.m_invI * b.m_torque;
    b.m_linearVelocity.Multiply(b2Math.Clamp(1 - step.dt * b.m_linearDamping, 0, 1));
    b.m_angularVelocity *= b2Math.Clamp(1 - step.dt * b.m_angularDamping, 0, 1)
  }
  this.m_contactSolver.Initialize(step, m_contacts, m_contactCount, this.m_allocator);
  var contactSolver = this.m_contactSolver;
  contactSolver.InitVelocityConstraints(step);
  for(i = 0;i < m_jointCount;++i) {
    joint = m_joints[i];
    joint.InitVelocityConstraints(step)
  }
  for(i = 0;i < step.velocityIterations;++i) {
    for(j = 0;j < m_jointCount;++j) {
      joint = m_joints[j];
      joint.SolveVelocityConstraints(step)
    }
    contactSolver.SolveVelocityConstraints()
  }
  for(i = 0;i < m_jointCount;++i) {
    joint = m_joints[i];
    joint.FinalizeVelocityConstraints()
  }
  contactSolver.FinalizeVelocityConstraints();
  for(i = 0;i < m_bodyCount;++i) {
    b = m_bodies[i];
    if(b.GetType() == b2Body.b2_staticBody) {
      continue
    }
    var translationX = step.dt * b.m_linearVelocity.x;
    var translationY = step.dt * b.m_linearVelocity.y;
    if(translationX * translationX + translationY * translationY > b2Settings.b2_maxTranslationSquared) {
      b.m_linearVelocity.Normalize();
      b.m_linearVelocity.x *= b2Settings.b2_maxTranslation * step.inv_dt;
      b.m_linearVelocity.y *= b2Settings.b2_maxTranslation * step.inv_dt
    }
    var rotation = step.dt * b.m_angularVelocity;
    if(rotation * rotation > b2Settings.b2_maxRotationSquared) {
      if(b.m_angularVelocity < 0) {
        b.m_angularVelocity = -b2Settings.b2_maxRotation * step.inv_dt
      }else {
        b.m_angularVelocity = b2Settings.b2_maxRotation * step.inv_dt
      }
    }
    b.m_sweep.c0.SetV(b.m_sweep.c);
    b.m_sweep.a0 = b.m_sweep.a;
    b.m_sweep.c.x += step.dt * b.m_linearVelocity.x;
    b.m_sweep.c.y += step.dt * b.m_linearVelocity.y;
    b.m_sweep.a += step.dt * b.m_angularVelocity;
    b.SynchronizeTransform()
  }
  for(i = 0;i < step.positionIterations;++i) {
    var contactsOkay = contactSolver.SolvePositionConstraints(b2Settings.b2_contactBaumgarte);
    var jointsOkay = true;
    for(j = 0;j < m_jointCount;++j) {
      joint = m_joints[j];
      var jointOkay = joint.SolvePositionConstraints(b2Settings.b2_contactBaumgarte);
      jointsOkay = jointsOkay && jointOkay
    }
    if(contactsOkay && jointsOkay) {
      break
    }
  }
  this.Report(contactSolver.m_constraints);
  if(allowSleep) {
    var minSleepTime = Number.MAX_VALUE;
    var linTolSqr = b2Settings.b2_linearSleepTolerance * b2Settings.b2_linearSleepTolerance;
    var angTolSqr = b2Settings.b2_angularSleepTolerance * b2Settings.b2_angularSleepTolerance;
    for(i = 0;i < m_bodyCount;++i) {
      b = m_bodies[i];
      if(b.GetType() == b2Body.b2_staticBody) {
        continue
      }
      if((b.m_flags & b2Body.e_allowSleepFlag) == 0) {
        b.m_sleepTime = 0;
        minSleepTime = 0
      }
      if((b.m_flags & b2Body.e_allowSleepFlag) == 0 || b.m_angularVelocity * b.m_angularVelocity > angTolSqr || b2Math.Dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr) {
        b.m_sleepTime = 0;
        minSleepTime = 0
      }else {
        b.m_sleepTime += step.dt;
        minSleepTime = b2Math.Min(minSleepTime, b.m_sleepTime)
      }
    }
    if(minSleepTime >= b2Settings.b2_timeToSleep) {
      for(i = 0;i < m_bodyCount;++i) {
        b = m_bodies[i];
        b.SetAwake(false)
      }
    }
  }
};
b2Island.prototype.SolveTOI = function(subStep) {
  var i = 0;
  var j = 0;
  this.m_contactSolver.Initialize(subStep, m_contacts, m_contactCount, this.m_allocator);
  var contactSolver = this.m_contactSolver;
  for(i = 0;i < m_jointCount;++i) {
    m_joints[i].InitVelocityConstraints(subStep)
  }
  for(i = 0;i < subStep.velocityIterations;++i) {
    contactSolver.SolveVelocityConstraints();
    for(j = 0;j < m_jointCount;++j) {
      m_joints[j].SolveVelocityConstraints(subStep)
    }
  }
  for(i = 0;i < m_bodyCount;++i) {
    var b = m_bodies[i];
    if(b.GetType() == b2Body.b2_staticBody) {
      continue
    }
    var translationX = subStep.dt * b.m_linearVelocity.x;
    var translationY = subStep.dt * b.m_linearVelocity.y;
    if(translationX * translationX + translationY * translationY > b2Settings.b2_maxTranslationSquared) {
      b.m_linearVelocity.Normalize();
      b.m_linearVelocity.x *= b2Settings.b2_maxTranslation * subStep.inv_dt;
      b.m_linearVelocity.y *= b2Settings.b2_maxTranslation * subStep.inv_dt
    }
    var rotation = subStep.dt * b.m_angularVelocity;
    if(rotation * rotation > b2Settings.b2_maxRotationSquared) {
      if(b.m_angularVelocity < 0) {
        b.m_angularVelocity = -b2Settings.b2_maxRotation * subStep.inv_dt
      }else {
        b.m_angularVelocity = b2Settings.b2_maxRotation * subStep.inv_dt
      }
    }
    b.m_sweep.c0.SetV(b.m_sweep.c);
    b.m_sweep.a0 = b.m_sweep.a;
    b.m_sweep.c.x += subStep.dt * b.m_linearVelocity.x;
    b.m_sweep.c.y += subStep.dt * b.m_linearVelocity.y;
    b.m_sweep.a += subStep.dt * b.m_angularVelocity;
    b.SynchronizeTransform()
  }
  var k_toiBaumgarte = 0.75;
  for(i = 0;i < subStep.positionIterations;++i) {
    var contactsOkay = contactSolver.SolvePositionConstraints(k_toiBaumgarte);
    var jointsOkay = true;
    for(j = 0;j < m_jointCount;++j) {
      var jointOkay = m_joints[j].SolvePositionConstraints(b2Settings.b2_contactBaumgarte);
      jointsOkay = jointsOkay && jointOkay
    }
    if(contactsOkay && jointsOkay) {
      break
    }
  }
  this.Report(contactSolver.m_constraints)
};
b2Island.prototype.Report = function(constraints) {
  if(this.m_listener == null) {
    return
  }
  for(var i = 0;i < m_contactCount;++i) {
    var c = m_contacts[i];
    var cc = constraints[i];
    for(var j = 0;j < cc.pointCount;++j) {
      b2Island.s_impulse.normalImpulses[j] = cc.points[j].normalImpulse;
      b2Island.s_impulse.tangentImpulses[j] = cc.points[j].tangentImpulse
    }
    this.m_listener.PostSolve(c, b2Island.s_impulse)
  }
};
b2Island.prototype.AddBody = function(body) {
  body.m_islandIndex = m_bodyCount;
  m_bodies[m_bodyCount++] = body
};
b2Island.prototype.AddContact = function(contact) {
  m_contacts[m_contactCount++] = contact
};
b2Island.prototype.AddJoint = function(joint) {
  m_joints[m_jointCount++] = joint
};var b2PolyAndEdgeContact = function() {
  b2Contact.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2PolyAndEdgeContact.prototype, b2Contact.prototype);
b2PolyAndEdgeContact.prototype._super = function() {
  b2Contact.prototype.__constructor.apply(this, arguments)
};
b2PolyAndEdgeContact.prototype.__constructor = function() {
};
b2PolyAndEdgeContact.prototype.__varz = function() {
};
b2PolyAndEdgeContact.Create = function(allocator) {
  return new b2PolyAndEdgeContact
};
b2PolyAndEdgeContact.Destroy = function(contact, allocator) {
};
b2PolyAndEdgeContact.prototype.b2CollidePolyAndEdge = function(manifold, polygon, xf1, edge, xf2) {
};
b2PolyAndEdgeContact.prototype.Reset = function(fixtureA, fixtureB) {
  this._super.Reset(fixtureA, fixtureB);
  b2Settings.b2Assert(fixtureA.GetType() == b2Shape.e_polygonShape);
  b2Settings.b2Assert(fixtureB.GetType() == b2Shape.e_edgeShape)
};var b2PolyAndCircleContact = function() {
  b2Contact.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2PolyAndCircleContact.prototype, b2Contact.prototype);
b2PolyAndCircleContact.prototype._super = function() {
  b2Contact.prototype.__constructor.apply(this, arguments)
};
b2PolyAndCircleContact.prototype.__constructor = function() {
};
b2PolyAndCircleContact.prototype.__varz = function() {
};
b2PolyAndCircleContact.Create = function(allocator) {
  return new b2PolyAndCircleContact
};
b2PolyAndCircleContact.Destroy = function(contact, allocator) {
};
b2PolyAndCircleContact.prototype.Reset = function(fixtureA, fixtureB) {
  this._super.Reset(fixtureA, fixtureB);
  b2Settings.b2Assert(fixtureA.GetType() == b2Shape.e_polygonShape);
  b2Settings.b2Assert(fixtureB.GetType() == b2Shape.e_circleShape)
};var b2ContactPoint = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ContactPoint.prototype.__constructor = function() {
};
b2ContactPoint.prototype.__varz = function() {
  this.position = new b2Vec2;
  this.velocity = new b2Vec2;
  this.normal = new b2Vec2;
  this.id = new b2ContactID
};
b2ContactPoint.prototype.shape1 = null;
b2ContactPoint.prototype.shape2 = null;
b2ContactPoint.prototype.position = new b2Vec2;
b2ContactPoint.prototype.velocity = new b2Vec2;
b2ContactPoint.prototype.normal = new b2Vec2;
b2ContactPoint.prototype.separation = null;
b2ContactPoint.prototype.friction = null;
b2ContactPoint.prototype.restitution = null;
b2ContactPoint.prototype.id = new b2ContactID;var b2CircleContact = function() {
  b2Contact.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2CircleContact.prototype, b2Contact.prototype);
b2CircleContact.prototype._super = function() {
  b2Contact.prototype.__constructor.apply(this, arguments)
};
b2CircleContact.prototype.__constructor = function() {
};
b2CircleContact.prototype.__varz = function() {
};
b2CircleContact.Create = function(allocator) {
  return new b2CircleContact
};
b2CircleContact.Destroy = function(contact, allocator) {
};
b2CircleContact.prototype.Reset = function(fixtureA, fixtureB) {
  this._super.Reset(fixtureA, fixtureB)
};var b2World = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2World.prototype.__constructor = function(gravity, doSleep) {
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
  this.m_inv_dt0 = 0;
  m_contactManager.m_world = this;
  var bd = new b2BodyDef;
  m_groundBody = this.CreateBody(bd)
};
b2World.prototype.__varz = function() {
  this.s_stack = new Array;
  this.m_contactSolver = new b2ContactSolver;
  this.m_island = new b2Island
};
b2World.s_timestep2 = new b2TimeStep;
b2World.s_backupA = new b2Sweep;
b2World.s_backupB = new b2Sweep;
b2World.s_timestep = new b2TimeStep;
b2World.s_queue = new Array;
b2World.e_newFixture = 1;
b2World.e_locked = 2;
b2World.s_xf = new b2Transform;
b2World.s_jointColor = new b2Color(0.5, 0.8, 0.8);
b2World.m_warmStarting = null;
b2World.m_continuousPhysics = null;
b2World.prototype.s_stack = new Array;
b2World.prototype.m_contactSolver = new b2ContactSolver;
b2World.prototype.m_island = new b2Island;
b2World.prototype.m_jointList = null;
b2World.prototype.m_bodyCount = 0;
b2World.prototype.m_jointCount = 0;
b2World.prototype.m_controllerList = null;
b2World.prototype.m_controllerCount = 0;
b2World.prototype.m_gravity = null;
b2World.prototype.m_allowSleep = null;
b2World.prototype.m_destructionListener = null;
b2World.prototype.m_debugDraw = null;
b2World.prototype.m_inv_dt0 = null;
b2World.prototype.SetDestructionListener = function(listener) {
  this.m_destructionListener = listener
};
b2World.prototype.SetContactFilter = function(filter) {
  m_contactManager.m_contactFilter = filter
};
b2World.prototype.SetContactListener = function(listener) {
  m_contactManager.m_contactListener = listener
};
b2World.prototype.SetDebugDraw = function(debugDraw) {
  this.m_debugDraw = debugDraw
};
b2World.prototype.SetBroadPhase = function(broadPhase) {
  var oldBroadPhase = m_contactManager.m_broadPhase;
  m_contactManager.m_broadPhase = broadPhase;
  for(var b = m_bodyList;b;b = b.m_next) {
    for(var f = b.m_fixtureList;f;f = f.m_next) {
      f.m_proxy = broadPhase.CreateProxy(oldBroadPhase.GetFatAABB(f.m_proxy), f)
    }
  }
};
b2World.prototype.Validate = function() {
  m_contactManager.m_broadPhase.Validate()
};
b2World.prototype.GetProxyCount = function() {
  return m_contactManager.m_broadPhase.GetProxyCount()
};
b2World.prototype.CreateBody = function(def) {
  if(this.IsLocked() == true) {
    return null
  }
  var b = new b2Body(def, this);
  b.m_prev = null;
  b.m_next = m_bodyList;
  if(m_bodyList) {
    m_bodyList.m_prev = b
  }
  m_bodyList = b;
  ++this.m_bodyCount;
  return b
};
b2World.prototype.DestroyBody = function(b) {
  if(this.IsLocked() == true) {
    return
  }
  var jn = b.m_jointList;
  while(jn) {
    var jn0 = jn;
    jn = jn.next;
    if(this.m_destructionListener) {
      this.m_destructionListener.SayGoodbyeJoint(jn0.joint)
    }
    this.DestroyJoint(jn0.joint)
  }
  var coe = b.m_controllerList;
  while(coe) {
    var coe0 = coe;
    coe = coe.nextController;
    coe0.controller.RemoveBody(b)
  }
  var ce = b.m_contactList;
  while(ce) {
    var ce0 = ce;
    ce = ce.next;
    m_contactManager.Destroy(ce0.contact)
  }
  b.m_contactList = null;
  var f = b.m_fixtureList;
  while(f) {
    var f0 = f;
    f = f.m_next;
    if(this.m_destructionListener) {
      this.m_destructionListener.SayGoodbyeFixture(f0)
    }
    f0.DestroyProxy(m_contactManager.m_broadPhase);
    f0.Destroy()
  }
  b.m_fixtureList = null;
  b.m_fixtureCount = 0;
  if(b.m_prev) {
    b.m_prev.m_next = b.m_next
  }
  if(b.m_next) {
    b.m_next.m_prev = b.m_prev
  }
  if(b == m_bodyList) {
    m_bodyList = b.m_next
  }
  --this.m_bodyCount
};
b2World.prototype.CreateJoint = function(def) {
  var j = b2Joint.Create(def, null);
  j.m_prev = null;
  j.m_next = this.m_jointList;
  if(this.m_jointList) {
    this.m_jointList.m_prev = j
  }
  this.m_jointList = j;
  ++this.m_jointCount;
  j.m_edgeA.joint = j;
  j.m_edgeA.other = j.m_bodyB;
  j.m_edgeA.prev = null;
  j.m_edgeA.next = j.m_bodyA.m_jointList;
  if(j.m_bodyA.m_jointList) {
    j.m_bodyA.m_jointList.prev = j.m_edgeA
  }
  j.m_bodyA.m_jointList = j.m_edgeA;
  j.m_edgeB.joint = j;
  j.m_edgeB.other = j.m_bodyA;
  j.m_edgeB.prev = null;
  j.m_edgeB.next = j.m_bodyB.m_jointList;
  if(j.m_bodyB.m_jointList) {
    j.m_bodyB.m_jointList.prev = j.m_edgeB
  }
  j.m_bodyB.m_jointList = j.m_edgeB;
  var bodyA = def.bodyA;
  var bodyB = def.bodyB;
  if(def.collideConnected == false) {
    var edge = bodyB.GetContactList();
    while(edge) {
      if(edge.other == bodyA) {
        edge.contact.FlagForFiltering()
      }
      edge = edge.next
    }
  }
  return j
};
b2World.prototype.DestroyJoint = function(j) {
  var collideConnected = j.m_collideConnected;
  if(j.m_prev) {
    j.m_prev.m_next = j.m_next
  }
  if(j.m_next) {
    j.m_next.m_prev = j.m_prev
  }
  if(j == this.m_jointList) {
    this.m_jointList = j.m_next
  }
  var bodyA = j.m_bodyA;
  var bodyB = j.m_bodyB;
  bodyA.SetAwake(true);
  bodyB.SetAwake(true);
  if(j.m_edgeA.prev) {
    j.m_edgeA.prev.next = j.m_edgeA.next
  }
  if(j.m_edgeA.next) {
    j.m_edgeA.next.prev = j.m_edgeA.prev
  }
  if(j.m_edgeA == bodyA.m_jointList) {
    bodyA.m_jointList = j.m_edgeA.next
  }
  j.m_edgeA.prev = null;
  j.m_edgeA.next = null;
  if(j.m_edgeB.prev) {
    j.m_edgeB.prev.next = j.m_edgeB.next
  }
  if(j.m_edgeB.next) {
    j.m_edgeB.next.prev = j.m_edgeB.prev
  }
  if(j.m_edgeB == bodyB.m_jointList) {
    bodyB.m_jointList = j.m_edgeB.next
  }
  j.m_edgeB.prev = null;
  j.m_edgeB.next = null;
  b2Joint.Destroy(j, null);
  --this.m_jointCount;
  if(collideConnected == false) {
    var edge = bodyB.GetContactList();
    while(edge) {
      if(edge.other == bodyA) {
        edge.contact.FlagForFiltering()
      }
      edge = edge.next
    }
  }
};
b2World.prototype.AddController = function(c) {
  c.m_next = this.m_controllerList;
  c.m_prev = null;
  this.m_controllerList = c;
  c.m_world = this;
  this.m_controllerCount++;
  return c
};
b2World.prototype.RemoveController = function(c) {
  if(c.m_prev) {
    c.m_prev.m_next = c.m_next
  }
  if(c.m_next) {
    c.m_next.m_prev = c.m_prev
  }
  if(this.m_controllerList == c) {
    this.m_controllerList = c.m_next
  }
  this.m_controllerCount--
};
b2World.prototype.CreateController = function(controller) {
  if(controller.m_world != this) {
    throw new Error("Controller can only be a member of one world");
  }
  controller.m_next = this.m_controllerList;
  controller.m_prev = null;
  if(this.m_controllerList) {
    this.m_controllerList.m_prev = controller
  }
  this.m_controllerList = controller;
  ++this.m_controllerCount;
  controller.m_world = this;
  return controller
};
b2World.prototype.DestroyController = function(controller) {
  controller.Clear();
  if(controller.m_next) {
    controller.m_next.m_prev = controller.m_prev
  }
  if(controller.m_prev) {
    controller.m_prev.m_next = controller.m_next
  }
  if(controller == this.m_controllerList) {
    this.m_controllerList = controller.m_next
  }
  --this.m_controllerCount
};
b2World.prototype.SetWarmStarting = function(flag) {
  b2World.m_warmStarting = flag
};
b2World.prototype.SetContinuousPhysics = function(flag) {
  b2World.m_continuousPhysics = flag
};
b2World.prototype.GetBodyCount = function() {
  return this.m_bodyCount
};
b2World.prototype.GetJointCount = function() {
  return this.m_jointCount
};
b2World.prototype.GetContactCount = function() {
  return m_contactCount
};
b2World.prototype.SetGravity = function(gravity) {
  this.m_gravity = gravity
};
b2World.prototype.GetGravity = function() {
  return this.m_gravity
};
b2World.prototype.GetGroundBody = function() {
  return m_groundBody
};
b2World.prototype.Step = function(dt, velocityIterations, positionIterations) {
  if(m_flags & b2World.e_newFixture) {
    m_contactManager.FindNewContacts();
    m_flags &= ~b2World.e_newFixture
  }
  m_flags |= b2World.e_locked;
  var step = b2World.s_timestep2;
  step.dt = dt;
  step.velocityIterations = velocityIterations;
  step.positionIterations = positionIterations;
  if(dt > 0) {
    step.inv_dt = 1 / dt
  }else {
    step.inv_dt = 0
  }
  step.dtRatio = this.m_inv_dt0 * dt;
  step.warmStarting = b2World.m_warmStarting;
  m_contactManager.Collide();
  if(step.dt > 0) {
    Solve(step)
  }
  if(m_continuousPhysics && step.dt > 0) {
    SolveTOI(step)
  }
  if(step.dt > 0) {
    this.m_inv_dt0 = step.inv_dt
  }
  m_flags &= ~b2World.e_locked
};
b2World.prototype.ClearForces = function() {
  for(var body = m_bodyList;body;body = body.m_next) {
    body.m_force.SetZero();
    body.m_torque = 0
  }
};
b2World.prototype.DrawDebugData = function() {
  if(this.m_debugDraw == null) {
    return
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
  var b1 = new b2AABB;
  var b2 = new b2AABB;
  var vs = [new b2Vec2, new b2Vec2, new b2Vec2, new b2Vec2];
  var color = new b2Color(0, 0, 0);
  if(flags & b2DebugDraw.e_shapeBit) {
    for(b = m_bodyList;b;b = b.m_next) {
      xf = b.m_xf;
      for(f = b.GetFixtureList();f;f = f.m_next) {
        s = f.GetShape();
        if(b.IsActive() == false) {
          color.Set(0.5, 0.5, 0.3);
          DrawShape(s, xf, color)
        }else {
          if(b.GetType() == b2Body.b2_staticBody) {
            color.Set(0.5, 0.9, 0.5);
            DrawShape(s, xf, color)
          }else {
            if(b.GetType() == b2Body.b2_kinematicBody) {
              color.Set(0.5, 0.5, 0.9);
              DrawShape(s, xf, color)
            }else {
              if(b.IsAwake() == false) {
                color.Set(0.6, 0.6, 0.6);
                DrawShape(s, xf, color)
              }else {
                color.Set(0.9, 0.7, 0.7);
                DrawShape(s, xf, color)
              }
            }
          }
        }
      }
    }
  }
  if(flags & b2DebugDraw.e_jointBit) {
    for(j = this.m_jointList;j;j = j.m_next) {
      DrawJoint(j)
    }
  }
  if(flags & b2DebugDraw.e_controllerBit) {
    for(var c = this.m_controllerList;c;c = c.m_next) {
      c.Draw(this.m_debugDraw)
    }
  }
  if(flags & b2DebugDraw.e_pairBit) {
    color.Set(0.3, 0.9, 0.9);
    for(var contact = m_contactManager.m_contactList;contact;contact = contact.GetNext()) {
      var fixtureA = contact.GetFixtureA();
      var fixtureB = contact.GetFixtureB();
      var cA = fixtureA.GetAABB().GetCenter();
      var cB = fixtureB.GetAABB().GetCenter();
      this.m_debugDraw.DrawSegment(cA, cB, color)
    }
  }
  if(flags & b2DebugDraw.e_aabbBit) {
    bp = m_contactManager.m_broadPhase;
    vs = [new b2Vec2, new b2Vec2, new b2Vec2, new b2Vec2];
    for(b = m_bodyList;b;b = b.GetNext()) {
      if(b.IsActive() == false) {
        continue
      }
      for(f = b.GetFixtureList();f;f = f.GetNext()) {
        var aabb = bp.GetFatAABB(f.m_proxy);
        vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
        vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
        vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
        vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);
        this.m_debugDraw.DrawPolygon(vs, 4, color)
      }
    }
  }
  if(flags & b2DebugDraw.e_centerOfMassBit) {
    for(b = m_bodyList;b;b = b.m_next) {
      xf = b2World.s_xf;
      xf.R = b.m_xf.R;
      xf.position = b.GetWorldCenter();
      this.m_debugDraw.DrawTransform(xf)
    }
  }
};
b2World.prototype.QueryAABB = function(callback, aabb) {
  var broadPhase = m_contactManager.m_broadPhase;
  function WorldQueryWrapper(proxy) {
    return callback(broadPhase.GetUserData(proxy))
  }
  broadPhase.Query(WorldQueryWrapper, aabb)
};
b2World.prototype.QueryShape = function(callback, shape, transform) {
  if(transform == null) {
    transform = new b2Transform;
    transform.SetIdentity()
  }
  var broadPhase = m_contactManager.m_broadPhase;
  function WorldQueryWrapper(proxy) {
    var fixture = broadPhase.GetUserData(proxy);
    if(b2Shape.TestOverlap(shape, transform, fixture.GetShape(), fixture.GetBody().GetTransform())) {
      return callback(fixture)
    }
    return true
  }
  var aabb = new b2AABB;
  shape.ComputeAABB(aabb, transform);
  broadPhase.Query(WorldQueryWrapper, aabb)
};
b2World.prototype.QueryPoint = function(callback, p) {
  var broadPhase = m_contactManager.m_broadPhase;
  function WorldQueryWrapper(proxy) {
    var fixture = broadPhase.GetUserData(proxy);
    if(fixture.TestPoint(p)) {
      return callback(fixture)
    }
    return true
  }
  var aabb = new b2AABB;
  aabb.lowerBound.Set(p.x - b2Settings.b2_linearSlop, p.y - b2Settings.b2_linearSlop);
  aabb.upperBound.Set(p.x + b2Settings.b2_linearSlop, p.y + b2Settings.b2_linearSlop);
  broadPhase.Query(WorldQueryWrapper, aabb)
};
b2World.prototype.RayCast = function(callback, point1, point2) {
  var broadPhase = m_contactManager.m_broadPhase;
  var output = new b2RayCastOutput;
  function RayCastWrapper(input, proxy) {
    var userData = broadPhase.GetUserData(proxy);
    var fixture = userData;
    var hit = fixture.RayCast(output, input);
    if(hit) {
      var fraction = output.fraction;
      var point = new b2Vec2((1 - fraction) * point1.x + fraction * point2.x, (1 - fraction) * point1.y + fraction * point2.y);
      return callback(fixture, point, output.normal, fraction)
    }
    return input.maxFraction
  }
  var input = new b2RayCastInput(point1, point2);
  broadPhase.RayCast(RayCastWrapper, input)
};
b2World.prototype.RayCastOne = function(point1, point2) {
  var result;
  function RayCastOneWrapper(fixture, point, normal, fraction) {
    result = fixture;
    return fraction
  }
  this.RayCast(RayCastOneWrapper, point1, point2);
  return result
};
b2World.prototype.RayCastAll = function(point1, point2) {
  var result = new Array;
  function RayCastAllWrapper(fixture, point, normal, fraction) {
    result[result.length] = fixture;
    return 1
  }
  this.RayCast(RayCastAllWrapper, point1, point2);
  return result
};
b2World.prototype.GetBodyList = function() {
  return m_bodyList
};
b2World.prototype.GetJointList = function() {
  return this.m_jointList
};
b2World.prototype.GetContactList = function() {
  return m_contactList
};
b2World.prototype.IsLocked = function() {
  return(m_flags & b2World.e_locked) > 0
};var b2EdgeAndCircleContact = function() {
  b2Contact.prototype.__varz.call(this);
  this.__varz();
  this.__constructor.apply(this, arguments)
};
extend(b2EdgeAndCircleContact.prototype, b2Contact.prototype);
b2EdgeAndCircleContact.prototype._super = function() {
  b2Contact.prototype.__constructor.apply(this, arguments)
};
b2EdgeAndCircleContact.prototype.__constructor = function() {
};
b2EdgeAndCircleContact.prototype.__varz = function() {
};
b2EdgeAndCircleContact.Create = function(allocator) {
  return new b2EdgeAndCircleContact
};
b2EdgeAndCircleContact.Destroy = function(contact, allocator) {
};
b2EdgeAndCircleContact.prototype.b2CollideEdgeAndCircle = function(manifold, edge, xf1, circle, xf2) {
};
b2EdgeAndCircleContact.prototype.Reset = function(fixtureA, fixtureB) {
  this._super.Reset(fixtureA, fixtureB)
};var b2ContactManager = function() {
  this.__varz();
  this.__constructor.apply(this, arguments)
};
b2ContactManager.prototype.__constructor = function() {
  m_world = null;
  m_contactCount = 0;
  m_contactFilter = b2ContactFilter.b2_defaultFilter;
  m_contactListener = b2ContactListener.b2_defaultListener;
  m_contactFactory = new b2ContactFactory(m_allocator);
  m_broadPhase = new b2DynamicTreeBroadPhase
};
b2ContactManager.prototype.__varz = function() {
};
b2ContactManager.s_evalCP = new b2ContactPoint;
b2ContactManager.prototype.AddPair = function(proxyUserDataA, proxyUserDataB) {
  var fixtureA = proxyUserDataA;
  var fixtureB = proxyUserDataB;
  var bodyA = fixtureA.GetBody();
  var bodyB = fixtureB.GetBody();
  if(bodyA == bodyB) {
    return
  }
  var edge = bodyB.GetContactList();
  while(edge) {
    if(edge.other == bodyA) {
      var fA = edge.contact.GetFixtureA();
      var fB = edge.contact.GetFixtureB();
      if(fA == fixtureA && fB == fixtureB) {
        return
      }
      if(fA == fixtureB && fB == fixtureA) {
        return
      }
    }
    edge = edge.next
  }
  if(bodyB.ShouldCollide(bodyA) == false) {
    return
  }
  if(m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
    return
  }
  var c = m_contactFactory.Create(fixtureA, fixtureB);
  fixtureA = c.GetFixtureA();
  fixtureB = c.GetFixtureB();
  bodyA = fixtureA.m_body;
  bodyB = fixtureB.m_body;
  c.m_prev = null;
  c.m_next = m_world.m_contactList;
  if(m_world.m_contactList != null) {
    m_world.m_contactList.m_prev = c
  }
  m_world.m_contactList = c;
  c.m_nodeA.contact = c;
  c.m_nodeA.other = bodyB;
  c.m_nodeA.prev = null;
  c.m_nodeA.next = bodyA.m_contactList;
  if(bodyA.m_contactList != null) {
    bodyA.m_contactList.prev = c.m_nodeA
  }
  bodyA.m_contactList = c.m_nodeA;
  c.m_nodeB.contact = c;
  c.m_nodeB.other = bodyA;
  c.m_nodeB.prev = null;
  c.m_nodeB.next = bodyB.m_contactList;
  if(bodyB.m_contactList != null) {
    bodyB.m_contactList.prev = c.m_nodeB
  }
  bodyB.m_contactList = c.m_nodeB;
  ++m_world.m_contactCount;
  return
};
b2ContactManager.prototype.FindNewContacts = function() {
  m_broadPhase.UpdatePairs(this.AddPair)
};
b2ContactManager.prototype.Destroy = function(c) {
  var fixtureA = c.GetFixtureA();
  var fixtureB = c.GetFixtureB();
  var bodyA = fixtureA.GetBody();
  var bodyB = fixtureB.GetBody();
  if(c.IsTouching()) {
    m_contactListener.EndContact(c)
  }
  if(c.m_prev) {
    c.m_prev.m_next = c.m_next
  }
  if(c.m_next) {
    c.m_next.m_prev = c.m_prev
  }
  if(c == m_world.m_contactList) {
    m_world.m_contactList = c.m_next
  }
  if(c.m_nodeA.prev) {
    c.m_nodeA.prev.next = c.m_nodeA.next
  }
  if(c.m_nodeA.next) {
    c.m_nodeA.next.prev = c.m_nodeA.prev
  }
  if(c.m_nodeA == bodyA.m_contactList) {
    bodyA.m_contactList = c.m_nodeA.next
  }
  if(c.m_nodeB.prev) {
    c.m_nodeB.prev.next = c.m_nodeB.next
  }
  if(c.m_nodeB.next) {
    c.m_nodeB.next.prev = c.m_nodeB.prev
  }
  if(c.m_nodeB == bodyB.m_contactList) {
    bodyB.m_contactList = c.m_nodeB.next
  }
  m_contactFactory.Destroy(c);
  --m_contactCount
};
b2ContactManager.prototype.Collide = function() {
  var c = m_world.m_contactList;
  while(c) {
    var fixtureA = c.GetFixtureA();
    var fixtureB = c.GetFixtureB();
    var bodyA = fixtureA.GetBody();
    var bodyB = fixtureB.GetBody();
    if(bodyA.IsAwake() == false && bodyB.IsAwake() == false) {
      c = c.GetNext();
      continue
    }
    if(c.m_flags & b2Contact.e_filterFlag) {
      if(bodyB.ShouldCollide(bodyA) == false) {
        var cNuke = c;
        c = cNuke.GetNext();
        this.Destroy(cNuke);
        continue
      }
      if(m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
        cNuke = c;
        c = cNuke.GetNext();
        this.Destroy(cNuke);
        continue
      }
      c.m_flags &= ~b2Contact.e_filterFlag
    }
    var proxyA = fixtureA.m_proxy;
    var proxyB = fixtureB.m_proxy;
    var overlap = m_broadPhase.TestOverlap(proxyA, proxyB);
    if(overlap == false) {
      cNuke = c;
      c = cNuke.GetNext();
      this.Destroy(cNuke);
      continue
    }
    c.Update(m_contactListener);
    c = c.GetNext()
  }
};if(typeof exports !== "undefined") {
  exports.b2BoundValues = b2BoundValues;
  exports.b2Math = b2Math;
  exports.b2DistanceOutput = b2DistanceOutput;
  exports.b2Mat33 = b2Mat33;
  exports.b2ContactPoint = b2ContactPoint;
  exports.b2PairManager = b2PairManager;
  exports.b2PositionSolverManifold = b2PositionSolverManifold;
  exports.b2OBB = b2OBB;
  exports.b2CircleContact = b2CircleContact;
  exports.b2PulleyJoint = b2PulleyJoint;
  exports.b2Pair = b2Pair;
  exports.b2TimeStep = b2TimeStep;
  exports.b2FixtureDef = b2FixtureDef;
  exports.b2World = b2World;
  exports.b2PrismaticJoint = b2PrismaticJoint;
  exports.b2Controller = b2Controller;
  exports.b2ContactID = b2ContactID;
  exports.b2RevoluteJoint = b2RevoluteJoint;
  exports.b2JointDef = b2JointDef;
  exports.b2Transform = b2Transform;
  exports.b2GravityController = b2GravityController;
  exports.b2EdgeAndCircleContact = b2EdgeAndCircleContact;
  exports.b2EdgeShape = b2EdgeShape;
  exports.b2BuoyancyController = b2BuoyancyController;
  exports.b2LineJointDef = b2LineJointDef;
  exports.b2Contact = b2Contact;
  exports.b2DistanceJoint = b2DistanceJoint;
  exports.b2Body = b2Body;
  exports.b2DestructionListener = b2DestructionListener;
  exports.b2PulleyJointDef = b2PulleyJointDef;
  exports.b2ContactEdge = b2ContactEdge;
  exports.b2ContactConstraint = b2ContactConstraint;
  exports.b2ContactImpulse = b2ContactImpulse;
  exports.b2DistanceJointDef = b2DistanceJointDef;
  exports.b2ContactResult = b2ContactResult;
  exports.b2EdgeChainDef = b2EdgeChainDef;
  exports.b2Vec2 = b2Vec2;
  exports.b2Vec3 = b2Vec3;
  exports.b2DistanceProxy = b2DistanceProxy;
  exports.b2FrictionJointDef = b2FrictionJointDef;
  exports.b2PolygonContact = b2PolygonContact;
  exports.b2TensorDampingController = b2TensorDampingController;
  exports.b2ContactFactory = b2ContactFactory;
  exports.b2WeldJointDef = b2WeldJointDef;
  exports.b2ConstantAccelController = b2ConstantAccelController;
  exports.b2GearJointDef = b2GearJointDef;
  exports.ClipVertex = ClipVertex;
  exports.b2SeparationFunction = b2SeparationFunction;
  exports.b2ManifoldPoint = b2ManifoldPoint;
  exports.b2Color = b2Color;
  exports.b2PolygonShape = b2PolygonShape;
  exports.b2DynamicTreePair = b2DynamicTreePair;
  exports.b2ContactConstraintPoint = b2ContactConstraintPoint;
  exports.b2FrictionJoint = b2FrictionJoint;
  exports.b2ContactFilter = b2ContactFilter;
  exports.b2ControllerEdge = b2ControllerEdge;
  exports.b2Distance = b2Distance;
  exports.b2Fixture = b2Fixture;
  exports.b2DynamicTreeNode = b2DynamicTreeNode;
  exports.b2MouseJoint = b2MouseJoint;
  exports.b2DistanceInput = b2DistanceInput;
  exports.b2BodyDef = b2BodyDef;
  exports.b2DynamicTreeBroadPhase = b2DynamicTreeBroadPhase;
  exports.b2Settings = b2Settings;
  exports.b2Proxy = b2Proxy;
  exports.b2Point = b2Point;
  exports.b2BroadPhase = b2BroadPhase;
  exports.b2Manifold = b2Manifold;
  exports.b2WorldManifold = b2WorldManifold;
  exports.b2PrismaticJointDef = b2PrismaticJointDef;
  exports.b2RayCastOutput = b2RayCastOutput;
  exports.b2ConstantForceController = b2ConstantForceController;
  exports.b2TimeOfImpact = b2TimeOfImpact;
  exports.b2CircleShape = b2CircleShape;
  exports.b2MassData = b2MassData;
  exports.b2Joint = b2Joint;
  exports.b2GearJoint = b2GearJoint;
  exports.b2DynamicTree = b2DynamicTree;
  exports.b2JointEdge = b2JointEdge;
  exports.b2LineJoint = b2LineJoint;
  exports.b2NullContact = b2NullContact;
  exports.b2ContactListener = b2ContactListener;
  exports.b2RayCastInput = b2RayCastInput;
  exports.b2TOIInput = b2TOIInput;
  exports.Features = Features;
  exports.b2FilterData = b2FilterData;
  exports.b2Island = b2Island;
  exports.b2ContactManager = b2ContactManager;
  exports.b2ContactSolver = b2ContactSolver;
  exports.b2Simplex = b2Simplex;
  exports.b2AABB = b2AABB;
  exports.b2Jacobian = b2Jacobian;
  exports.b2Bound = b2Bound;
  exports.b2RevoluteJointDef = b2RevoluteJointDef;
  exports.b2PolyAndEdgeContact = b2PolyAndEdgeContact;
  exports.b2SimplexVertex = b2SimplexVertex;
  exports.b2WeldJoint = b2WeldJoint;
  exports.b2Collision = b2Collision;
  exports.b2Mat22 = b2Mat22;
  exports.b2SimplexCache = b2SimplexCache;
  exports.b2PolyAndCircleContact = b2PolyAndCircleContact;
  exports.b2MouseJointDef = b2MouseJointDef;
  exports.b2Shape = b2Shape;
  exports.b2Segment = b2Segment;
  exports.b2ContactRegister = b2ContactRegister;
  exports.b2DebugDraw = b2DebugDraw;
  exports.b2Sweep = b2Sweep
}
;
