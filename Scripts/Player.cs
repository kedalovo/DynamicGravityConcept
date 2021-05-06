using Godot;
using System;
using System.Collections.Generic;
public class Player : RigidBody
{
	private Spatial World;
	private Label VectorLabel;
	private Label FloorLabel;
	private Label GravityLabel;
	private Area Gravitator;
	private MeshInstance GravitatorVisualiser;
	private MeshInstance CorrectorVisualiser;
	private MeshInstance ForceVisualiser;

	private MeshInstance North;
	private MeshInstance South;
	private MeshInstance East;
	private MeshInstance West;

	private MeshInstance ControlPlane;

	private MeshInstance NorthPlain;
	private MeshInstance SouthPlain;
	private MeshInstance EastPlain;
	private MeshInstance WestPlain;

	private Vector3 ApproximatedVector;
	private Vector3 Corrector;
	private Vector3 InitialGravity;
	private Vector3 Rotator;
	private List<Vector3> AllContacts;
	public int MovementSpeed = 5;

	public bool IsOnFloor;
	public bool IsOnNose;
	public override void _Ready()
	{
		SetPhysicsProcess(false);

		World = GetTree().Root.GetNode<Spatial>("World");
		VectorLabel = World.GetNode<Label>("Camera/Control/Panel/VBoxContainer/VectorLabel");
		FloorLabel = World.GetNode<Label>("Camera/Control/Panel/VBoxContainer/FloorLabel");
		GravityLabel = World.GetNode<Label>("Camera/Control/Panel/VBoxContainer/GravityLabel");
		Gravitator = GetNode<Area>("Gravitator");
		GravitatorVisualiser = GetNode<MeshInstance>("Visualiser1");
		CorrectorVisualiser = GetNode<MeshInstance>("Visualiser2");
		ForceVisualiser = GetNode<MeshInstance>("Visualiser3");

		North = GetNode<MeshInstance>("North");
		South = GetNode<MeshInstance>("South");
		East = GetNode<MeshInstance>("East");
		West = GetNode<MeshInstance>("West");

		ControlPlane = GetNode<MeshInstance>("ControlPlain");

		NorthPlain = ControlPlane.GetNode<MeshInstance>("NorthPlain");
		SouthPlain = ControlPlane.GetNode<MeshInstance>("SouthPlain");
		EastPlain = ControlPlane.GetNode<MeshInstance>("EastPlain");
		WestPlain = ControlPlane.GetNode<MeshInstance>("WestPlain");

		Corrector = Vector3.Zero;
		Rotator = Vector3.Zero;
		InitialGravity = new Vector3(0, -1, 0);

		SetPhysicsProcess(true);
	}
	public override void _PhysicsProcess(float delta)
	{
		if (Input.IsActionJustPressed("ui_esc")) GetTree().Quit();
		Corrector = InitialGravity.DirectionTo(Gravitator.GravityVec);
		GravitatorVisualiser.Translation = Gravitator.GravityVec;
		CorrectorVisualiser.Translation = Corrector;
		ControlPlane.RotationDegrees = CalculateRotation(InitialGravity, Gravitator.GravityVec.Normalized()) * 90;
	}
	public override void _IntegrateForces(PhysicsDirectBodyState state)
	{
		North.Translation = (NorthPlain.GlobalTransform.origin - GlobalTransform.origin).Normalized() * MovementSpeed - Gravitator.GravityVec.Normalized();
		South.Translation = (SouthPlain.GlobalTransform.origin - GlobalTransform.origin).Normalized() * MovementSpeed - Gravitator.GravityVec.Normalized();
		East.Translation = (EastPlain.GlobalTransform.origin - GlobalTransform.origin).Normalized() * MovementSpeed - Gravitator.GravityVec.Normalized();
		West.Translation = (WestPlain.GlobalTransform.origin - GlobalTransform.origin).Normalized() * MovementSpeed - Gravitator.GravityVec.Normalized();
		if (Input.IsActionPressed("ui_up")) AddCentralForce(North.Translation);
		if (Input.IsActionPressed("ui_down")) AddCentralForce(South.Translation);
		if (Input.IsActionPressed("ui_right")) AddCentralForce(East.Translation);
		if (Input.IsActionPressed("ui_left")) AddCentralForce(West.Translation);

		ForceVisualiser.Translation = state.LinearVelocity;

		int Contacts = state.GetContactCount();
		if (Contacts > 0)
		{
			AllContacts = new List<Vector3>();
			for (int i = 0; i < Contacts; i++) AllContacts.Add(state.GetContactLocalNormal(i));
			ApproximatedVector = Approximate(AllContacts);
			Gravitator.GravityVec = -ApproximatedVector;
		}

		VectorLabel.Text = "Vector: " + state.LinearVelocity.ToString();
		FloorLabel.Text = "On Floor: " + IsOnFloor + '\n' + "On Nose: " + IsOnNose;
		GravityLabel.Text = "Gravity: " + state.TotalGravity + '\n' + Contacts + '\n' + ApproximatedVector + '\n' + Corrector;
	}
	private Vector3 Approximate(List<Vector3> Contacts)
	{
		Vector3 Result = Contacts[0];

		for (int i = 1; i < Contacts.Count; i++) Result += Contacts[i];

		return Result / Contacts.Count;
	}
	private Vector3 CalculateRotation(Vector3 A, Vector3 B)
	{
		Vector3 Result = Vector3.Zero;

		//if (A == B) return A;

		Vector3 C = A.Cross(B);
		float x = C.x;
		float y = C.y;
		float z = C.z;
		float N = (float)Mathf.Acos(A.Dot(B) / (A.Length() * B.Length()));
		Result.y = Mathf.Atan2(y * Mathf.Sin(N) - x * z * (1 - Mathf.Cos(N)), 1 - (Mathf.Pow(y, 2) + Mathf.Pow(z, 2)) * (1 - Mathf.Cos(N)));
		Result.z = Mathf.Asin(x * y * (1 - Mathf.Cos(N)) + z * Mathf.Sin(N));
		Result.x = Mathf.Atan2(x * Mathf.Sin(N) - y * z * (1 - Mathf.Cos(N)), 1 - (Mathf.Pow(x, 2) + Mathf.Pow(z, 2)) * (1 - Mathf.Cos(N)));

		return Result;
	}
}
