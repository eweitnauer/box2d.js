var b2Proxy = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Proxy.prototype.__constructor = function(){}
b2Proxy.prototype.__varz = function(){
this.lowerBounds =  new Vector(2);
this.upperBounds =  new Vector(2);
this.pairs =  new Dictionary();
}
// static attributes
// static methods
// attributes
b2Proxy.prototype.lowerBounds =  new Vector(2);
b2Proxy.prototype.upperBounds =  new Vector(2);
b2Proxy.prototype.overlapCount =  0;
b2Proxy.prototype.timeStamp =  0;
b2Proxy.prototype.pairs =  new Dictionary();
b2Proxy.prototype.next =  null;
b2Proxy.prototype.userData =  null;
// methods
b2Proxy.prototype.IsValid = function () { return this.overlapCount != b2BroadPhase.b2_invalid; }