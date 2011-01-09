var b2ConstantForceController = function() {
b2Controller.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2ConstantForceController.prototype, b2Controller.prototype)
b2ConstantForceController.prototype._super = function(){ b2Controller.prototype.__constructor.apply(this, arguments) }
b2ConstantForceController.prototype.__constructor = function(){}
b2ConstantForceController.prototype.__varz = function(){
this.F =  new b2Vec2(0,0);
}
// static attributes
// static methods
// attributes
b2ConstantForceController.prototype.F =  new b2Vec2(0,0);
// methods
b2ConstantForceController.prototype.Step = function (step) {
		for(var i=m_bodyList;i;i=i.nextBody){
			var body = i.body;
			if(!body.IsAwake())
				continue;
			body.ApplyForce(this.F,body.GetWorldCenter());
		}
	}